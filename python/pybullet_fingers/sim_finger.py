#!/usr/bin/env python3
# -------------------------------------------------------------------------------------------------
# The documentation in this code is heavily derived from the official documentation of PyBullet at
# https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit# among other scattered sources.
# -------------------------------------------------------------------------------------------------
import os
import time
import math
import random
import numpy as np

import pybullet
import pybullet_data

import rospkg

from pybullet_fingers.observation import Observation

class Finger:
    """
    A simulation environment for the single finger robot.
    This environment is based on PyBullet, the official Python wrapper around the
    Bullet-C API.
    :param time_step: It is the time between two simulation steps. Defaults to
    1./240. Don't set this to be larger than 1./60.
    :type time_step: float
    :param visual_debugging: Set this to 'True' for a GUI interface to
    the simulation.
    :type visual_debugging: bool
    :param finger_tip: Define the link index of the end-effector. Set this looking
    at the output of print_link_information.
    :type finger_tip: int
    :param revolute_joint_ids: Define the range of movable joints to which control
    commands will be sent. Set this looking at the output of print_joint_information.
    :type revolute_joint_ids: class 'range'
    :param finger_link_ids: Define the range of links that corresponds to the
    upper, middle, and lower links of the finger. Set this looking at the output
    of print_link_information.
    :type finger_link_ids: class 'range'
    :param action_repetitions: The number of times the simulation will be stepped
    for a single control command.
    :type action_repetitions: int
    :param max_motor_torque: The maximum allowed torque that can be applied to
    any of the joint motors.
    :type max_motor_torque: float
    :param buffer_size: The size of the buffer for which actions are stored. 1
    by design.
    :type buffer_size: int
    """

    def __init__(self,
                 time_step=0.001,
                 visual_debugging=True,
                 finger_type="single"):
        """
        Constructor, initializes the physical world we will work in.
        """
        self.visual_debugging = visual_debugging
        self.time_step = time_step
        self.finger_type = finger_type
        self.position_gain = 20
        self.velocity_gain = 7
        self.safety_kd = 1.4
        self.max_motor_torque = 0.36

        self.action_index = -1
        self.observation_index = 0


        self.set_finger_type_dependency()
        self.connect_to_simulation()
        self.make_physical_world()
        self.disable_velocity_control()

    def set_finger_type_dependency(self):
        '''
        Sets the paths for the URDFs to use depending upon the finger type
        '''
        self.robot_properties_path = os.path.join(rospkg.RosPack().get_path("robot_properties_manipulator"))

        if "single" in self.finger_type:
            self.finger_urdf_path = os.path.join(self.robot_properties_path,
                                                 "urdf", "finger.urdf")
            self.stage_meshfile_path = os.path.join(self.robot_properties_path,
                                                    "meshes", "stl", "Stage_simplified.stl")
            self.stage_meshscale = [1, 1, 1]


        elif "tri" in self.finger_type:
            self.finger_urdf_path = os.path.join(self.robot_properties_path,
                                                 "urdf", "trifinger.urdf")
            self.stage_meshfile_path = os.path.join(self.robot_properties_path,
                                                    "meshes", "stl", "BL-M_Table_ASM_big.stl")
            self.stage_meshscale = [0.001, 0.001, 0.001]



    def connect_to_simulation(self):
        """
        Connect to the GUI physics server for visual debugging via a GUI
        interface, or enbable DIRECT connection.

        .. note::
           Both GUI and DIRECT physics servers (the server) execute the
           simulation and rendering in the same process as PyBullet(the client).
           To connect to a physics server in a different process on the same
           machine, or on a remote machine, use SHARED_MEMORY, UDP, or
           TCP networking.

           In GUI connection mode, use ctrl or alt with mouse scroll to adjust
           the view of the camera.
        """

        if self.visual_debugging:
            pybullet.connect(pybullet.GUI)
        else:
            pybullet.connect(pybullet.DIRECT)

    def make_physical_world(self):
        """
        Set parameters of the world such as gravity, time for which a
        simulation step should run etc, and load the finger and the
        interaction objects.
        """
        pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
        pybullet.setGravity(0, 0, -9.81)
        pybullet.setTimeStep(self.time_step)

        plane_id = pybullet.loadURDF("plane.urdf", [0, 0, 0])
        self.import_finger_model()
        self.set_dynamics_properties()
        self.import_non_convex_shapes()
        self.import_interaction_objects()

    def import_finger_model(self):
        """
        Load a physics model of the finger from its urdf.

        ..note::
          use bitwise | instead of logical or as mentioned in the official
          documentation for the urdf flags.

        :return: Unique id of the body specified in the URDF (non-negative if
        the URDF can be loaded).
        :rtype: int
        """

        finger_base_position = [0, 0, 0.05]
        finger_base_orientation = pybullet.getQuaternionFromEuler([0, 0, 0])

        self.finger_id = pybullet.loadURDF(
            fileName=self.finger_urdf_path,
            basePosition=finger_base_position,
            baseOrientation=finger_base_orientation,
            useFixedBase=1,
            flags=pybullet.URDF_USE_INERTIA_FROM_FILE | pybullet.URDF_USE_SELF_COLLISION)

        self.get_indices()
        self.last_joint_position = [0] * len(self.revolute_joint_ids)


    def get_indices(self):
        '''
        Since the indices of the revolute joints and the tips are different in different URDFs,
        this function sets the indices, using the current URDF, for:
         - finger links
         - revolute joints
         - tip joints
        '''
        joint_indices = []
        tip_indices = []
        joint_names = ["finger_upper_link", "finger_middle_link", "finger_lower_link"]
        tip_names = ["finger_tip"]

        for id in range(pybullet.getNumJoints(self.finger_id)):
            link_name = pybullet.getJointInfo(self.finger_id, id)[12].decode('UTF-8') # index 12 has the link name
            if any(name in link_name for name in joint_names):
                joint_indices.append(id)
            if any(name in link_name for name in tip_names):
                tip_indices.append(id)

        # TODO the revolute_joint_ids and the finger_link_ids are pointing to the same indices.
        self.revolute_joint_ids = joint_indices
        self.finger_link_ids = joint_indices
        self.finger_tip_ids = tip_indices


    def import_non_convex_shapes(self):
        """
        Imports the non-convex arena (stage).
        """
        stage_id = pybullet.createCollisionShape(
            shapeType=pybullet.GEOM_MESH,
            fileName=self.stage_meshfile_path,
            meshScale=self.stage_meshscale,
            flags=pybullet.GEOM_FORCE_CONCAVE_TRIMESH)
        stage_position = [0, 0, 0.05]
        stage_orientation = pybullet.getQuaternionFromEuler([0, 0, 0])
        stage = pybullet.createMultiBody(
            baseCollisionShapeIndex=stage_id,
            baseVisualShapeIndex=-1,
            basePosition=stage_position,
            baseOrientation=stage_orientation)

    def set_dynamics_properties(self):
        """
        To change properties of the robot such as its mass, friction, damping,
        maximum joint velocities etc.
        """
        for link_id in self.finger_link_ids:
            pybullet.changeDynamics(
                bodyUniqueId=self.finger_id,
                linkIndex=link_id,
                maxJointVelocity=1e9,
                restitution=0.8,
                jointDamping=0.0,
                lateralFriction=0.1,
                spinningFriction=0.1,
                rollingFriction=0.1,
                linearDamping=0.5,
                angularDamping=0.5,
                contactStiffness=0.1,
                contactDamping=0.05)

    def print_link_information(self):
        """
        Get the link indices along with their names as defined in the urdf.

        ..note::
          Source: https://pybullet.org/Bullet/phpBB3/viewtopic.php?t=12728.

          (Single finger output for reference)
          0 finger_base_link
          1 finger_upper_holder_link
          2 finger_upper_link
          3 finger_middle_link
          4 finger_lower_link
          5 finger_tip_link
        """

        link_name_to_index = {
            pybullet.getBodyInfo(
                self.finger_id)[0].decode('UTF-8'): -1, }

        for id in range(pybullet.getNumJoints(self.finger_id)):
            link_name = pybullet.getJointInfo(self.finger_id, id)[12].decode('UTF-8')
            link_name_to_index[link_name] = id
            print(link_name_to_index[link_name], link_name)

    def print_joint_information(self):
        """
        Get the joint indices along with their names as defined in the urdf.

        ..note::
          Source: https://pybullet.org/Bullet/phpBB3/viewtopic.php?t=12728.

          (Single finger output for reference)
          0 base_to_finger
          1 finger_base_to_holder
          2 finger_base_to_upper_joint
          3 finger_upper_to_middle_joint
          4 finger_middle_to_lower_joint
          5 finger_lower_to_tip_joint
        """

        link_name_to_index = {
            pybullet.getBodyInfo(
                self.finger_id)[0].decode('UTF-8'): -1, }

        for id in range(pybullet.getNumJoints(self.finger_id)):
            joint_name = pybullet.getJointInfo(self.finger_id, id)[1].decode('UTF-8')
            link_name_to_index[joint_name] = id
            print(link_name_to_index[joint_name], joint_name)

    def disable_velocity_control(self):
        """
        To disable the high friction velocity motors created by
        default at all revolute and prismatic joints while loading them from
        the urdf.
        """

        pybullet.setJointMotorControlArray(bodyUniqueId=self.finger_id,
                                           jointIndices=self.revolute_joint_ids,
                                           controlMode=pybullet.VELOCITY_CONTROL,
                                           targetVelocities=[0] * len(self.revolute_joint_ids),
                                           forces=[0] * len(self.revolute_joint_ids))

    def sample_random_position_in_arena(self):
        """
        Set a new position in the arena for the interaction object, which
        the finger has to reach.

        :return: The random position of the target set in the arena.
        :rtype: list of 3 floats
        """

        angle_x = random.uniform(-2 * math.pi, 2 * math.pi)
        angle_y = random.uniform(-2 * math.pi, 2 * math.pi)
        radial_distance = random.uniform(0, 0.22)

        object_position = [radial_distance * math.cos(angle_x),
                           radial_distance * math.sin(angle_y),
                           0.08]
        return object_position

    def display_object_at_target(self, object_position):
        """
        Remove the older block and display a new one at the new position.

        :param object_position: Position in the xyz coordinate
        system at which the object is to be displayed.
        :type object_position: vec3/list of 3 floats
        """

        pybullet.removeBody(self.block)
        self.block = pybullet.createMultiBody(
            baseCollisionShapeIndex=self.block_id,
            basePosition=object_position,
            baseMass=self.block_mass)

    def import_interaction_objects(self):
        """
        Import any object that the finger interacts/has to interact with.
        """
        block_position = [0.15, 0., 0.08]
        block_size = [0.03, 0.03, 0.03]
        self.block_mass = 0.25

        self.block_id = pybullet.createCollisionShape(
            shapeType=pybullet.GEOM_BOX, halfExtents=block_size)
        self.block = pybullet.createMultiBody(
            baseCollisionShapeIndex=self.block_id,
            basePosition=block_position,
            baseMass=self.block_mass)

    def inverse_kinematics(self, desired_tip_position):
        """
        Compute the joint angular positions needed to get to reach the block.

        :param desired_tip_position: (vec3/list of 3 floats) Target position in the
        xyz coordinate system of the end-effector.
        :type desired_tip_position: vec3/list of 3 floats
        :return: The required positions of the joints to reach the target position.
        :rtype: list of lists of 3 floats
        """

        at_target_threshold = 0.001
        joint_pos = []
        for idx, finger_tip_id in enumerate(self.finger_tip_ids):
            joint_pos += list(
                pybullet.calculateInverseKinematics(
                    bodyUniqueId=self.finger_id,
                    endEffectorLinkIndex=finger_tip_id,
                    targetPosition=desired_tip_position[idx],
                    residualThreshold=at_target_threshold))[3*idx : 3*idx + 3]

        # if the desired position is outside the bounds (i.e. not reachable),
        # the value for the related joints is set to NaN.
        # If this happens, we set the value to the last "valid" value to avoid error.
        if np.isnan(joint_pos).any():
            joint_pos = self.last_joint_position
        else:
            self.last_joint_position = joint_pos

        return joint_pos

    def append_desired_action(self, action):
        """
        Pass an action on which safety checks will be performed and then the
        action will be sent to the simulated finger.

        .. note::
        For now, only torques as actions are valid.

        :param action: The structure consisting of the torques to be applied.
        :type action: class 'robot_interfaces.py_finger_types.Action'
        :return time_index: The current time index in the buffer of length 1. 1.
        :rtype time_index: int
        """
        if not np.isnan(action.position).all():
            raise Exception('position control is not implemented yet')

        if self.action_index >= self.observation_index:
            raise Exception(
                'currently you have to call get_observation after each append_desired_action.')

        command = list(action.torque)
        self._set_motor_torques(command)

        self.action_index = self.action_index + 1
        return self.action_index

    def get_observation(self, time_index):
        """
        Get the observation (position/velocity/torque) of the joints
        corresponding to the current time index.

        :param time_index: The current time index in the buffer of length 1. 1.
        :type time_index: int
        :return observation: The Observation structure consisting of the position,
        velocity, and torques of the joints.
        :rtype observation: class 'robot_interfaces.py_finger_types.Observation'
        """
        if not time_index == self.action_index:
            raise Exception('currently you can only get the latest observation')

        assert(self.observation_index == self.action_index)

        observation = Observation()
        current_joint_states = pybullet.getJointStates(self.finger_id, self.revolute_joint_ids)

        observation.position = [joint[0] for joint in current_joint_states]
        observation.velocity = [joint[1] for joint in current_joint_states]
        observation.torque = [joint[3] for joint in current_joint_states]

        self._step_simulation()
        self.observation_index = self.observation_index + 1

        return observation

    def _apply_action(self, target_position, control_mode):
        """
        Specify the target position for the finger tip and the method of control
        (torque or position) to obtain the joint positions for reaching there
        and then applying the torque or the position control commands.

        :param target_position: a valid target position to which you want to send
        the finger-tip.
        :type target_position: list of 3 floats
        :param control_mode: specify if you want to control the finger using
        torque control or position control
        :type control_mode: string, valid strings = {"position", "torque"}
        """

        target_position = [target_position] if len(np.array(target_position).shape) != 2 else target_position

        joint_positions = self.inverse_kinematics(target_position)

        if control_mode == "torque":
            torque_commands = self.compute_pd_control_torques(joint_positions)
            self._set_motor_torques(torque_commands)
        elif control_mode == "position":
            self._pybullet_position_control(joint_positions)
        else:
            print('Invalid control_mode, enter either "position" or "torque".')

    def compute_pd_control_torques(self, joint_positions):
        """
        Specify the joint positions that the motors have to reach, and compute
        the torques to be applied to reach these positions using PD control.

        :param joint_positions: The desired joint positions.
        :type joint_positions: list of 3 floats
        :return The torques to be sent to the joints of the finger in order to
        reach the specified joint_positions.
        :rtype: list of 3 floats
        """

        current_joint_states = pybullet.getJointStates(self.finger_id, self.revolute_joint_ids)
        current_position = [joint[0] for joint in current_joint_states]
        current_velocity = [joint[1] for joint in current_joint_states]

        position_error = list(np.subtract(joint_positions, current_position))

        kp = [self.position_gain]*len(self.revolute_joint_ids)
        kd = [self.velocity_gain]*len(self.revolute_joint_ids)

        position_feedback = list(np.multiply(kp, position_error))
        velocity_feedback = list(np.multiply(kd, current_velocity))

        joint_torques = list(np.subtract(position_feedback, velocity_feedback))

        return joint_torques

    def _set_motor_torques(self, torque_commands):
        """
        Send torque commands to the motors directly.

        :param torque_commands: The forces to be sent to the movable joints of
        the motor.
        :type torque_commands: list of 3 floats
        """

        torque_commands = self._safety_torque_check(torque_commands)

        pybullet.setJointMotorControlArray(
            bodyUniqueId=self.finger_id,
            jointIndices=self.revolute_joint_ids,
            controlMode=pybullet.TORQUE_CONTROL,
            forces=torque_commands)

    def _pybullet_position_control(self, position_commands):
        """
        Perform position control on the finger (CONTROL_MODE_POSITION_VELOCITY_PD)

        :param joint_positions: The desired target position of the joints.
        :type joint_positions: list of floats
        """

        pybullet.setJointMotorControlArray(
            bodyUniqueId=self.finger_id,
            jointIndices=self.revolute_joint_ids,
            controlMode=pybullet.POSITION_CONTROL,
            targetPositions=position_commands,
            targetVelocities=[0] * len(self.revolute_joint_ids))

        current_joint_states = pybullet.getJointStates(self.finger_id, self.revolute_joint_ids)
        # printing the joint torques for debugging
        # print("Joint Torques:", [joint[3] for joint in current_joint_states])

    def _safety_torque_check(self, desired_torques):
        applied_torques = np.clip(np.asarray(desired_torques), -self.max_motor_torque, self.max_motor_torque)
        applied_torques = np.multiply(applied_torques, self.safety_kd)
        applied_torques = list(np.clip(np.asarray(desired_torques), -self.max_motor_torque, self.max_motor_torque))

        return applied_torques

    def _step_simulation(self):
        """
        Step the simulation to go to the next world state.
        """
        pybullet.stepSimulation()
