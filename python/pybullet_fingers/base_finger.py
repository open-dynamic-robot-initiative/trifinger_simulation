import os
import math
import random
import numpy as np

import pinocchio
import pybullet


class BaseFinger:
    """
    The SimFinger and RealFinger environments derive from this base
    class, which implements some common core functions.

    Args:
        enable_visualization (bool): Set this to 'True' for a GUI interface to
            the simulation.
        finger_type (string- "single"/ "tri"): Specify if you want to run the
            single finger model or the trifinger model, so finger_type expects
            one of the two values: "single" or "tri", respectively.
        action_bounds (dict): The limits of the action space used by the
            policy network.  Has to contain keys "low" and "high" with lists of
            limit values.
        sampling_strategy (string):  Strategy with which positions for the
            three fingers are sampled. Unused when using the single finger. Has
            to be one of the following values:

            - "separated": Samples for each finger a tip position somewhere
                  in this fingers section of the workspace.  This should
                  result in target positions that minimize the risk of
                  collisions between the fingers.
            - "uniform": Samples for each finger a position uniformly over the
                  whole joint range.
            - "triangle": Samples a position somewhere in the workspace and
                  places the tips of the free fingers around it with fixed
                  distance.
    """

    def __init__(self,
                 finger_type,
                 action_bounds,
                 enable_visualization,
                 sampling_strategy):
        """
        Constructor sets up the requirements for either of the single or
        the trifinger robots.
        """
        self.enable_visualization = enable_visualization
        self.finger_type = finger_type
        self.action_bounds = action_bounds
        self.sampling_strategy = sampling_strategy

        self.set_finger_type_dependency()
        self.init_joint_lists()
        self.connect_to_simulation()
        self.pinocchio_finger_init()

    def connect_to_simulation(self):
        """
        Connect to the Pybullet client via either GUI (visual rendering
        enabled) or DIRECT (no visual rendering) physics servers.

        In GUI connection mode, use ctrl or alt with mouse scroll to adjust
        the view of the camera.
        """
        if not self.enable_simulation:
            return

        if self.enable_visualization:
            pybullet.connect(pybullet.GUI)
        else:
            pybullet.connect(pybullet.DIRECT)

    def pinocchio_finger_init(self):
        """
        Initialize the robot model and data in pinocchio from the urdf
        """
        self.pinocchio_robot_model = pinocchio.buildModelFromUrdf(
            self.finger_urdf_path)
        self.pinocchio_robot_data = self.pinocchio_robot_model.createData()
        self.pinocchio_tip_link_ids = [
            self.pinocchio_robot_model.getFrameId(link_name)
            for link_name in self.tip_link_names
        ]

    def set_finger_type_dependency(self):
        """
        Sets the paths for the URDFs to use depending upon the finger type
        """
        try:
            import rospkg
            self.robot_properties_path = rospkg.RosPack().get_path(
                "robot_properties_fingers")
        except Exception:
            print("Importing the robot description files from local copy "
                  "of the robot_properties_fingers package.")
            self.robot_properties_path = os.path.join(
                                         os.path.dirname(__file__),
                                         "robot_properties_fingers")

        if "single" in self.finger_type:
            self.finger_urdf_path = os.path.join(self.robot_properties_path,
                                                 "urdf",
                                                 "finger.urdf")
            self.number_of_fingers = 1

        elif "tri" in self.finger_type:
            self.finger_urdf_path = os.path.join(self.robot_properties_path,
                                                 "urdf",
                                                 "trifinger.urdf")
            self.number_of_fingers = 3
        else:
            raise ValueError("Finger type has to be one of 'single' or 'tri'")

    def init_joint_lists(self):
        """
        Initialize lists of link/joint names depending on which robot is used.
        """
        if self.number_of_fingers == 1:
            self.joint_names = [
                "finger_upper_link",
                "finger_middle_link",
                "finger_lower_link",
            ]
            self.tip_link_names = ["finger_tip_link"]
        else:
            self.joint_names = [
                "finger_upper_link_0",
                "finger_middle_link_0",
                "finger_lower_link_0",
                "finger_upper_link_120",
                "finger_middle_link_120",
                "finger_lower_link_120",
                "finger_upper_link_240",
                "finger_middle_link_240",
                "finger_lower_link_240",
            ]
            self.tip_link_names = [
                "finger_tip_link_0",
                "finger_tip_link_120",
                "finger_tip_link_240",
            ]

    def display_goal(self):
        """
        Visualize the goal for reaching for each finger
        """
        if not self.enable_visualization:
            return

        block_size = [0.015] * 3

        color_cycle = [[1, 0, 0, 1],
                       [0, 1, 0, 1],
                       [0, 0, 1, 1]]

        goal_shape_ids = [None] * self.number_of_fingers
        self.goal_ids = [None] * self.number_of_fingers
        self.goal_orientations = [None] * self.number_of_fingers

        # Can use both a block, or a sphere: uncomment accordingly
        for i in range(self.number_of_fingers):
            color = color_cycle[i % len(color_cycle)]
            goal_shape_ids[i] = pybullet.createVisualShape(
                # shapeType=pybullet.GEOM_BOX,
                # halfExtents=block_size,
                shapeType=pybullet.GEOM_SPHERE,
                radius=block_size[0],
                rgbaColor=color)
            self.goal_ids[i] = pybullet.createMultiBody(
                baseVisualShapeIndex=goal_shape_ids[i],
                basePosition=[0.18, 0.18, 0.08],
                baseOrientation=[0, 0, 0, 1])
            _, self.goal_orientations[i] = \
                pybullet.getBasePositionAndOrientation(self.goal_ids[i])

    def reset_goal_markers(self, goal_positions):
        """
        Set new positions for the goal markers.

        Args:
            goal_positions (list of lists):  List of tuples with
            x,y,z positions of all goals.
        """
        if not self.enable_visualization:
            return

        for goal_id, orientation, position in zip(self.goal_ids,
                                                  self.goal_orientations,
                                                  goal_positions):
            pybullet.resetBasePositionAndOrientation(goal_id,
                                                     position,
                                                     orientation)

    def import_finger_model(self):
        """
        Load the single/trifinger model from the corresponding urdf
        """
        if not self.enable_simulation:
            return

        finger_base_position = [0, 0, 0.]
        finger_base_orientation = pybullet.getQuaternionFromEuler([0, 0, 0])

        self.finger_id = pybullet.loadURDF(
            fileName=self.finger_urdf_path,
            basePosition=finger_base_position,
            baseOrientation=finger_base_orientation,
            useFixedBase=1,
            flags=(pybullet.URDF_USE_INERTIA_FROM_FILE |
                   pybullet.URDF_USE_SELF_COLLISION))

        # create a map link_name -> link_index
        # Source: https://pybullet.org/Bullet/phpBB3/viewtopic.php?t=12728.
        self.link_name_to_index = {
            pybullet.getBodyInfo(self.finger_id)[0].decode('UTF-8'): -1, }
        for joint_idx in range(pybullet.getNumJoints(self.finger_id)):
            link_name = pybullet.getJointInfo(
                self.finger_id, joint_idx)[12].decode('UTF-8')
            self.link_name_to_index[link_name] = joint_idx

        self.init_indices()
        self.last_joint_position = [0] * len(self.revolute_joint_ids)

    def init_indices(self):
        """
        Since the indices of the revolute joints and the tips are different in
        different URDFs, this function sets the indices, using the current
        URDF, for:
         - finger links
         - revolute joints
         - tip joints
        """
        # TODO naming: these are indices, not ids
        self.revolute_joint_ids = [self.link_name_to_index[name]
                                   for name in self.joint_names]
        self.finger_tip_ids = [self.link_name_to_index[name]
                               for name in self.tip_link_names]

        # joint and link indices are the same in pybullet
        # TODO do we even need this variable?
        self.finger_link_ids = self.revolute_joint_ids

    def import_object(self,
                      mesh_file_path,
                      position,
                      orientation=[0, 0, 0, 1],
                      is_concave=False,
                      color_rgba=None):
        """Create a collision object based on a mesh file.

        Args:
            mesh_file_path:  Path to the mesh file.
            position:  Position (x, y, z) of the object.
            orientation:  Quaternion defining the orientation of the object.
            is_concave:  If set to true, the object is loaded as concav shape.
                Only use this for static objects.
            color_rgba:  Optional colour of the object given as a list of RGBA
                values in the interval [0, 1].  If not specified, pyBullet
                assigns a random colour.

        Returns:
            The created object.
        """
        if is_concave:
            flags = pybullet.GEOM_FORCE_CONCAVE_TRIMESH
        else:
            flags = 0

        object_id = pybullet.createCollisionShape(
            shapeType=pybullet.GEOM_MESH,
            fileName=mesh_file_path,
            flags=flags)

        obj = pybullet.createMultiBody(
            baseCollisionShapeIndex=object_id,
            baseVisualShapeIndex=-1,
            basePosition=position,
            baseOrientation=orientation)

        # set colour
        if color_rgba is not None:
            pybullet.changeVisualShape(obj, -1, rgbaColor=color_rgba)

        return obj

    def create_stage(self, high_border=True):
        """Create the stage (table and boundary).

        Args:
            high_border:  Only used for the TriFinger.  If set to False, the
                old, low boundary will be loaded instead of the high one.
        """
        try:
            import rospkg
            self.robot_properties_path = rospkg.RosPack().get_path(
                "robot_properties_fingers")
        except Exception:
            print("Importing the robot description files from local copy "
                  "of the robot_properties_fingers package.")
            self.robot_properties_path = os.path.join(
                                         os.path.dirname(__file__),
                                         "robot_properties_fingers")

        def mesh_path(filename):
            return os.path.join(self.robot_properties_path,
                                "meshes",
                                "stl",
                                filename)

        if "single" in self.finger_type:
            self.import_object(mesh_path("Stage_simplified.stl"),
                               position=[0, 0, 0.01],
                               is_concave=True)

        elif "tri" in self.finger_type:
            table_colour = (0.31, 0.27, 0.25, 1.0)
            high_border_colour = (0.95, 0.95, 0.95, 1.0)
            if high_border:
                self.import_object(mesh_path("trifinger_table_without_border.stl"),
                                   position=[0, 0, 0.01],
                                   is_concave=False,
                                   color_rgba=table_colour)
                self.import_object(mesh_path("high_table_boundary.stl"),
                                   position=[0, 0, 0.01],
                                   is_concave=True,
                                   color_rgba=high_border_colour)
            else:
                self.import_object(mesh_path("BL-M_Table_ASM_big.stl"),
                                   position=[0, 0, 0.01],
                                   is_concave=True,
                                   color_rgba=table_colour)
        else:
            raise ValueError("Invalid finger type '%s'" % self.finger_type)



    def import_interaction_objects(self,
                                   size=0.065,
                                   position=[0.15, 0., 0.09],
                                   orientation=[0, 0, 0, 1]):
        """
        Import any object that the finger interacts/has to interact with.
        """
        block_position = position
        block_size = [size / 2., size / 2., size / 2.]
        block_orientation = orientation
        self.block_mass = 0.08

        self.block_id = pybullet.createCollisionShape(
            shapeType=pybullet.GEOM_BOX, halfExtents=block_size)
        self.block = pybullet.createMultiBody(
            baseCollisionShapeIndex=self.block_id,
            basePosition=block_position,
            baseOrientation=block_orientation,
            baseMass=self.block_mass)

    def set_block_state(self, position, orientation):
        """
        Resets the block state to the provided position and orientation
        """
        pybullet.resetBasePositionAndOrientation(
            self.block, position, orientation)

    def get_block_state(self):
        """
        Returns the current position and orientation of the block.
        """
        state = pybullet.getBasePositionAndOrientation(self.block)
        return np.array([state[0] + state[1]]).reshape(-1, 1)

    def remove_block(self):
        """
        Removes the block from the environment
        """
        pybullet.removeBody(self.block)

    def sample_random_joint_positions(self):
        """
        Sample a random joint configuration for each finger.

        Returns:
            Flat list of joint positions.
        """
        list_to_return = []
        for _ in range(self.number_of_fingers):
            upper = random.uniform(-math.radians(30), math.radians(30))
            middle = random.uniform(-math.radians(60), math.radians(60))
            lower = random.uniform(-math.radians(100), -math.radians(2))
            list_to_return += [upper, middle, lower]
        return list_to_return

    def sample_random_joint_positions_for_reaching(self):
        """
        Sample random joint configuration with low risk of collisions.

        For the single Finger, this just calls
        sample_random_joint_positions().

        For the TriFinger, the sampling strategy depends on
        self.sampling_strategy.

        Returns:
            Flat list of joint angles.
        """
        if self.sampling_strategy == "uniform":
            return self.sample_random_joint_positions()

        elif self.sampling_strategy == "triangle":
            if self.number_of_fingers == 1:
                raise RuntimeError("Sampling strategy 'triangle' cannot"
                                   " be used with a single finger.")
            random_position = self.sample_random_position_in_arena()
            tip_positions = self.get_tip_positions_around_position(
                random_position)
            joint_positions = self.inverse_kinematics(tip_positions)
            # The inverse kinematics is _very_ inaccurate, but as we anyway
            # are sampling random positions, we don't care so much for some
            # random deviation.  The placement of the goals for the single
            # fingers relative to each other should more or less be
            # preserved.
            return joint_positions

        elif self.sampling_strategy == "separated":
            def sample_point_in_angle_limits():
                while True:
                    joint_pos = np.random.uniform(low=[-np.pi / 2,
                                                       np.deg2rad(-77.5),
                                                       np.deg2rad(-172)],
                                                  high=[np.pi / 2,
                                                        np.deg2rad(257.5),
                                                        np.deg2rad(-2)])
                    tip_pos = self.forward_kinematics(
                        np.concatenate([
                            joint_pos for i in range(
                                self.number_of_fingers)
                        ]))[0]
                    dist_to_center = np.linalg.norm(tip_pos[:2])
                    angle = np.arccos(tip_pos[0] / dist_to_center)
                    if ((np.pi / 6 < angle < 5 / 6 * np.pi)
                            and (tip_pos[1] > 0)
                            and (0.02 < dist_to_center < 0.2)
                            and np.all((self.action_bounds["low"])[0:3]
                                       < joint_pos)
                            and np.all((self.action_bounds["high"])[0:3]
                                       > joint_pos)):
                        return joint_pos

            joint_positions = np.concatenate([
                sample_point_in_angle_limits()
                for i in range(self.number_of_fingers)
            ])

            return joint_positions

        else:
            raise ValueError("Invalid sampling strategy '{}'".format(
                self.sampling_strategy))

    def forward_kinematics(self, joint_positions):
        """
        Compute end effector positions for the given joint configuration.

        Args:
            joint_positions: Flat list of angular joint positions.

        Returns:
            List of end-effector positions. Each position is given as an
            np.array with x,y,z positions.
        """
        pinocchio.framesForwardKinematics(self.pinocchio_robot_model,
                                          self.pinocchio_robot_data,
                                          joint_positions)

        return [
            np.asarray(
                self.pinocchio_robot_data.oMf[link_id].translation
            ).reshape(-1).tolist()
            for link_id in self.pinocchio_tip_link_ids
        ]

    def sample_random_position_in_arena(
            self, angle_limits=(-2 * math.pi, 2 * math.pi),
            min_radius=0):
        """
        Set a new position in the arena for the interaction object, which
        the finger has to reach.

        Returns:
            The random position of the target set in the arena.
        """
        angle = random.uniform(*angle_limits)
        radial_distance = random.uniform(min_radius, 0.15)
        height_z = random.uniform(0.1, 0.2)

        object_position = [radial_distance * math.cos(angle),
                           radial_distance * math.sin(angle),
                           height_z]

        return object_position

    def get_tip_positions_around_position(self, position):
        """
        Compute finger tip positions close to the given target position

        For single finger, the tip position will be the same as the given
        position.  For the TriFinger, the tips of the three fingers will be
        placed around it with some distance to avoid collision.

        Args:
            position (array-like): The target x,y,z-position.

        Returns:
            tip_positions (list of array-like): List with one target position
                for each finger tip (each position given as a (x, y, z) tuple).
        """
        position = np.array(position)
        if self.number_of_fingers == 1:
            return [position]
        elif self.number_of_fingers == 3:
            angle = np.deg2rad(-120)
            rot_120 = np.array([[np.cos(angle), -np.sin(angle), 0],
                                [np.sin(angle), np.cos(angle), 0],
                                [0, 0, 1]])
            displacement = np.array([0.05, 0, 0])
            tip1 = position + displacement
            displacement = rot_120 @ displacement
            tip2 = position + displacement
            displacement = rot_120 @ displacement
            tip3 = position + displacement

            return [tip1, tip2, tip3]
        else:
            raise ValueError("Invalid number of fingers")
