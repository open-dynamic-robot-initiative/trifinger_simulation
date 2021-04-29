import copy
import os
import numpy as np
import typing

import pybullet
import pybullet_data

from trifinger_simulation.action import Action
from trifinger_simulation.observation import Observation
from trifinger_simulation import collision_objects
from trifinger_simulation import pinocchio_utils
from trifinger_simulation import finger_types_data


def int_to_rgba(color: int, alpha: int = 0xFF) -> typing.Tuple[float, ...]:
    """Convert an 24-bit integer to an rgba tuple.

    Converts color given as a single 24-bit integer (e.g. a hex value 0xFF0011)
    to an rgba tuple where each value is in the range [0, 1].

    Args:
        color: An RGB colour given as single number (e.g. 0xFF0011).
        alpha: Optional alpha value in the interval [0, 0xFF].

    Returns:
        tuple: The colour as a tuple (r, g, b, a) where each element is in
        [0.0, 1.0].
    """
    return tuple(
        (x & 0xFF) / 0xFF for x in (color >> 16, color >> 8, color, alpha)
    )


class SimFinger:
    """
    pyBullet simulation environment for the single and the tri-finger robots.
    """

    #: The kp gains for the pd control of the finger(s). Note, this depends
    #: on the simulation step size and has been set for a simulation rate
    #: of 250 Hz.
    position_gains: typing.Sequence[float]

    #: The kd gains for the pd control of the finger(s). Note, this depends
    #: on the simulation step size and has been set for a simulation rate
    #: of 250 Hz.
    velocity_gains: typing.Sequence[float]

    def __init__(
        self,
        finger_type,
        time_step=0.001,
        enable_visualization=False,
        robot_position_offset=(0, 0, 0),
    ):
        """
        Constructor, initializes the physical world we will work in.

        Args:
            finger_type (string): Name of the finger type.  Use
                :meth:`get_valid_finger_types` to get a list of all supported
                types.
            time_step (float): Time (in seconds) between two simulation steps.
                Don't set this to be larger than 1/60.  The gains etc. are set
                according to a time_step of 0.001 s.
            enable_visualization (bool): Set this to 'True' for a GUI interface
                to the simulation.
            robot_position_offset: Position offset with which the robot is
                placed in the world.  Use this, for example, to change the
                height of the fingers above the table.
        """
        self.finger_type = finger_types_data.check_finger_type(finger_type)
        self.number_of_fingers = finger_types_data.get_number_of_fingers(
            self.finger_type
        )

        self.time_step_s = time_step

        self.__set_default_pd_gains()

        #: The kd gains used for damping the joint motor velocities during the
        #: safety torque check on the joint motors.
        self.safety_kd = np.array([0.08, 0.08, 0.04] * self.number_of_fingers)

        #: The maximum allowable torque that can be applied to each motor.
        self.max_motor_torque = 0.396

        self._t = -1

        self.__create_link_lists()
        self.__set_urdf_path()
        self._pybullet_client_id = self.__connect_to_pybullet(
            enable_visualization
        )
        self.__setup_pybullet_simulation(robot_position_offset)

        self.kinematics = pinocchio_utils.Kinematics(
            self.finger_urdf_path, self.tip_link_names
        )

    def Action(self, torque=None, position=None):
        """
        Fill in the fields of the action structure.

        This is a factory go create an
        :class:`~trifinger_simulation.action.Action` instance with proper
        default values, depending on the finger type.

        Args:
            torque (array): Torques to apply to the joints.  Defaults to
                zero.
            position (array): Angular target positions for the joints.  If set
                to NaN for a joint, no position control is run for this joint.
                Defaults to all NaN.

        Returns:
            ~action.Action: the action to be applied to the motors
        """
        if torque is None:
            torque = np.array([0.0] * 3 * self.number_of_fingers)
        if position is None:
            position = np.array([np.nan] * 3 * self.number_of_fingers)

        action = Action(torque, position)

        return action

    def get_observation(self, t):
        """
        Get the observation at the time of
        applying the action, so the observation actually corresponds
        to the state of the environment due to the application of the
        previous action.

        This method steps the simulation!

        Args:
            t: Index of the time step.  The only valid value is the index of
                the current step (return value of the last call of
                :meth:`~append_desired_action`).

        Returns:
            Observation: Observation of the robot state

        Raises:
            ValueError: If invalid time index ``t`` is passed.
        """
        if t < 0:
            raise ValueError("Cannot access time index less than zero.")
        elif t == self._t:
            # observation from before action_t was applied
            observation = self._observation_t

        elif t == self._t + 1:
            # observation from after action_t was applied
            observation = self._get_latest_observation()

        else:
            raise ValueError(
                "You can only get the observation at the current time index,"
                " or the next one."
            )

        return observation

    def append_desired_action(self, action):
        """
        Pass an action on which safety checks
        will be performed and then the action will be applied to the motors.

        Args:
            action (~action.Action): Joint positions or torques or both

        Returns:
            int: The current time index t at which the action is applied.
        """
        # copy the action in a way that works for both Action and
        # robot_interfaces.(tri)finger.Action.  Note that a simple
        # copy.copy(action) does **not** work for robot_interfaces
        # actions!
        self._desired_action_t = type(action)(
            copy.copy(action.torque),
            copy.copy(action.position),
        )

        self._applied_action_t = self._set_desired_action(action)

        # save current observation, then step simulation
        self._observation_t = self._get_latest_observation()
        self._step_simulation()

        self._t += 1
        return self._t

    def get_desired_action(self, t):
        """Get the desired action of time step 't'.

        Args:
            t: Index of the time step.  The only valid value is the index of
                the current step (return value of the last call of
                :meth:`~append_desired_action`).

        Returns:
            The desired action of time step t.

        Raises:
            ValueError: If invalid time index ``t`` is passed.
        """
        self.__validate_time_index(t)
        return self._desired_action_t

    def get_applied_action(self, t):
        """Get the actually applied action of time step 't'.

        The actually applied action can differ from the desired one, e.g.
        because the position controller affects the torque command or because
        too big torques are clamped to the limits.

        Args:
            t: Index of the time step.  The only valid value is the index of
                the current step (return value of the last call of
                :meth:`~append_desired_action`).

        Returns:
            The applied action of time step t.

        Raises:
            ValueError: If invalid time index ``t`` is passed.
        """
        self.__validate_time_index(t)
        return self._applied_action_t

    def get_timestamp_ms(self, t):
        """Get timestamp of time step 't'.

        Args:
            t: Index of the time step.  The only valid value is the index of
                the current step (return value of the last call of
                :meth:`~append_desired_action`).

        Returns:
            Timestamp in milliseconds.  The timestamp starts at zero when
            initializing and is increased with every simulation step according
            to the configured time step.

        Raises:
            ValueError: If invalid time index ``t`` is passed.
        """
        if t < 0:
            raise ValueError("Cannot access time index less than zero.")
        elif t == self._t or t == self._t + 1:
            return self.time_step_s * 1000 * t
        else:
            raise ValueError(
                "Given time index t has to match with index of the current"
                " step or the next one."
            )

    def get_current_timeindex(self):
        """Get the current time index."""
        if self._t < 0:
            raise ValueError(
                "Time index is only available after sending the first action."
            )

        return self._t

    def reset_finger_positions_and_velocities(
        self, joint_positions, joint_velocities=None
    ):
        """
        Reset the finger(s) to have the desired joint positions (required)
        and joint velocities (defaults to all zero) "instantaneously", that
        is w/o calling the control loop.

        Args:
            joint_positions (array-like):  Angular position for each joint.
            joint_velocities (array-like): Angular velocities for each joint.
                If None, velocities are set to 0.
        """
        if joint_velocities is None:
            joint_velocities = [0] * self.number_of_fingers * 3

        for i, joint_id in enumerate(self.pybullet_joint_indices):
            pybullet.resetJointState(
                self.finger_id,
                joint_id,
                joint_positions[i],
                joint_velocities[i],
                physicsClientId=self._pybullet_client_id,
            )
        return self._get_latest_observation()

    def _get_latest_observation(self):
        """Get observation of the current state.

        Returns:
            observation (Observation): the joint positions, velocities, and
            torques of the joints.
        """
        observation = Observation()
        current_joint_states = pybullet.getJointStates(
            self.finger_id,
            self.pybullet_joint_indices,
            physicsClientId=self._pybullet_client_id,
        )

        observation.position = np.array(
            [joint[0] for joint in current_joint_states]
        )
        observation.velocity = np.array(
            [joint[1] for joint in current_joint_states]
        )
        # pybullet.getJointStates only contains actual joint torques in
        # POSITION_CONTROL and VELOCITY_CONTROL mode.  In TORQUE_CONTROL mode
        # only zeros are reported, the actual torque is exactly the same as the
        # one that was applied.
        try:
            observation.torque = copy.copy(self.__applied_torque)
        except AttributeError:
            # when called before any torque was applied (and thus
            # self.__applied_torque does not exist), set it to zero
            observation.torque = np.zeros(len(observation.velocity))

        finger_contact_states = [
            pybullet.getContactPoints(
                bodyA=self.finger_id,
                linkIndexA=tip,
                physicsClientId=self._pybullet_client_id,
            )
            for tip in self.pybullet_tip_link_indices
        ]
        tip_forces = []
        for i in range(len(finger_contact_states)):
            directed_contact_force = 0.0
            try:
                for contact_point in finger_contact_states[i]:
                    directed_contact_force += np.array(contact_point[9])
            except IndexError:
                pass
            tip_forces.append(directed_contact_force)
        observation.tip_force = np.array(tip_forces)

        # The measurement of the push sensor of the real robot lies in the
        # interval [0, 1].  It does not go completely to zero, so add a bit of
        # "no contact" offset.  It saturates somewhere around 5 N.
        push_sensor_saturation_force_N = 5.0
        push_sensor_no_contact_value = 0.05
        observation.tip_force /= push_sensor_saturation_force_N
        observation.tip_force += push_sensor_no_contact_value
        np.clip(observation.tip_force, 0.0, 1.0, out=observation.tip_force)

        return observation

    def _set_desired_action(self, desired_action):
        """Set the given action after performing safety checks.

        Args:
            desired_action (Action): Joint positions or torques or both

        Returns:
            applied_action:  The action that is actually applied after
            performing the safety checks.
        """
        # copy the action in a way that works for both Action and
        # robot_interfaces.(tri)finger.Action.  Note that a simple
        # copy.copy(desired_action) does **not** work for robot_interfaces
        # actions!
        applied_action = type(desired_action)(
            copy.copy(desired_action.torque),
            copy.copy(desired_action.position),
        )

        def set_gains(gains, defaults):
            """Replace NaN entries in gains with values from defaults."""
            mask = np.isnan(gains)
            output = copy.copy(gains)
            output[mask] = defaults[mask]
            return output

        applied_action.position_kp = set_gains(
            desired_action.position_kp, self.position_gains
        )
        applied_action.position_kd = set_gains(
            desired_action.position_kd, self.velocity_gains
        )

        torque_command = np.asarray(copy.copy(desired_action.torque))
        if not np.isnan(desired_action.position).all():
            torque_command += np.array(
                self.__compute_pd_control_torques(
                    desired_action.position,
                    applied_action.position_kp,
                    applied_action.position_kd,
                )
            )

        applied_action.torque = self.__safety_check_torques(torque_command)

        self.__set_pybullet_motor_torques(applied_action.torque)

        # store this here for use in _get_latest_observation()
        self.__applied_torque = applied_action.torque

        return applied_action

    def _step_simulation(self):
        """
        Step the simulation to go to the next world state.
        """
        pybullet.stepSimulation(
            physicsClientId=self._pybullet_client_id,
        )

    def _disconnect_from_pybullet(self):
        """Disconnect from the simulation.

        Disconnects from the simulation and sets simulation to disabled to
        avoid any further function calls to it.
        """
        if pybullet.isConnected(physicsClientId=self._pybullet_client_id):
            pybullet.disconnect(
                physicsClientId=self._pybullet_client_id,
            )

    def __set_pybullet_motor_torques(self, motor_torques):

        pybullet.setJointMotorControlArray(
            bodyUniqueId=self.finger_id,
            jointIndices=self.pybullet_joint_indices,
            controlMode=pybullet.TORQUE_CONTROL,
            forces=motor_torques,
            physicsClientId=self._pybullet_client_id,
        )

    def __safety_check_torques(self, desired_torques):
        """
        Perform a check on the torques being sent to be applied to
        the motors so that they do not exceed the safety torque limit

        Args:
            desired_torques (array): The torques desired to be
                applied to the motors

        Returns:
            applied_torques (array): The torques that can be actually
            applied to the motors (and will be applied)
        """
        applied_torques = np.clip(
            np.asarray(desired_torques),
            -self.max_motor_torque,
            +self.max_motor_torque,
        )

        current_joint_states = pybullet.getJointStates(
            self.finger_id,
            self.pybullet_joint_indices,
            physicsClientId=self._pybullet_client_id,
        )
        current_velocity = np.array(
            [joint[1] for joint in current_joint_states]
        )
        applied_torques -= self.safety_kd * current_velocity

        applied_torques = np.clip(
            np.asarray(applied_torques),
            -self.max_motor_torque,
            +self.max_motor_torque,
        )

        return applied_torques

    def __compute_pd_control_torques(self, joint_positions, kp=None, kd=None):
        """
        Compute torque command to reach given target position using a PD
        controller.

        Args:
            joint_positions (array-like, shape=(n,)):  Desired joint positions.
            kp (array-like, shape=(n,)): P-gains, one for each joint.
            kd (array-like, shape=(n,)): D-gains, one for each joint.

        Returns:
            List of torques to be sent to the joints of the finger in order to
            reach the specified joint_positions.
        """
        if kp is None:
            kp = self.position_gains
        if kd is None:
            kd = self.velocity_gains

        current_joint_states = pybullet.getJointStates(
            self.finger_id,
            self.pybullet_joint_indices,
            physicsClientId=self._pybullet_client_id,
        )
        current_position = np.array(
            [joint[0] for joint in current_joint_states]
        )
        current_velocity = np.array(
            [joint[1] for joint in current_joint_states]
        )

        position_error = joint_positions - current_position

        position_feedback = np.asarray(kp) * position_error
        velocity_feedback = np.asarray(kd) * current_velocity

        joint_torques = position_feedback - velocity_feedback

        # set nan entries to zero (nans occur on joints for which the target
        # position was set to nan)
        joint_torques[np.isnan(joint_torques)] = 0.0

        return joint_torques.tolist()

    def __validate_time_index(self, t):
        """Raise error if t does not match with self._t."""
        if t < 0:
            raise ValueError("Cannot access time index less than zero.")
        elif t != self._t:
            raise ValueError(
                "Given time index %d does not match with current index %d"
                % (t, self._t)
            )

    def __del__(self):
        """Clean up."""
        self._disconnect_from_pybullet()

    def __create_link_lists(self):
        """
        Initialize lists of link/joint names depending on which robot is used.
        """
        if self.number_of_fingers == 1:
            self.link_names = [
                "finger_upper_link",
                "finger_middle_link",
                "finger_lower_link",
            ]
            self.tip_link_names = ["finger_tip_link"]
        else:
            self.link_names = [
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

    def __setup_pybullet_simulation(self, robot_position_offset):
        """
        Set the physical parameters of the world in which the simulation
        will run, and import the models to be simulated

        Args:
            robot_position_offset: Position offset with which the robot is
                placed in the world.  Use this, for example, to change the
                height of the fingers above the table.
        """
        pybullet.setAdditionalSearchPath(
            pybullet_data.getDataPath(),
            physicsClientId=self._pybullet_client_id,
        )
        pybullet.setGravity(
            0,
            0,
            -9.81,
            physicsClientId=self._pybullet_client_id,
        )
        pybullet.setTimeStep(
            self.time_step_s, physicsClientId=self._pybullet_client_id
        )

        pybullet.loadURDF(
            "plane_transparent.urdf",
            [0, 0, -0.01],
            physicsClientId=self._pybullet_client_id,
        )
        self.__load_robot_urdf(robot_position_offset)
        self.__set_pybullet_params()
        self.__load_stage()
        self.__disable_pybullet_velocity_control()

    def __set_pybullet_params(self):
        """
        To change properties of the robot such as its mass, friction, damping,
        maximum joint velocities etc.
        """
        for link_id in self.pybullet_link_indices:
            pybullet.changeDynamics(
                bodyUniqueId=self.finger_id,
                linkIndex=link_id,
                maxJointVelocity=10,
                restitution=0.8,
                jointDamping=0.0,
                lateralFriction=0.1,
                spinningFriction=0.1,
                rollingFriction=0.1,
                linearDamping=0.5,
                angularDamping=0.5,
                contactStiffness=0.1,
                contactDamping=0.05,
                physicsClientId=self._pybullet_client_id,
            )

    def __disable_pybullet_velocity_control(self):
        """
        To disable the high friction velocity motors created by
        default at all revolute and prismatic joints while loading them from
        the urdf.
        """
        pybullet.setJointMotorControlArray(
            bodyUniqueId=self.finger_id,
            jointIndices=self.pybullet_joint_indices,
            controlMode=pybullet.VELOCITY_CONTROL,
            targetVelocities=[0] * len(self.pybullet_joint_indices),
            forces=[0] * len(self.pybullet_joint_indices),
            physicsClientId=self._pybullet_client_id,
        )

    @staticmethod
    def __connect_to_pybullet(enable_visualization):
        """
        Connect to the Pybullet client via either GUI (visual rendering
        enabled) or DIRECT (no visual rendering) physics servers.

        In GUI connection mode, use ctrl or alt with mouse scroll to adjust
        the view of the camera.
        """
        if enable_visualization:
            pybullet_client_id = pybullet.connect(pybullet.GUI)
        else:
            pybullet_client_id = pybullet.connect(pybullet.DIRECT)

        return pybullet_client_id

    def __set_urdf_path(self):
        """
        Sets the paths for the URDFs to use depending upon the finger type
        """
        try:
            from ament_index_python.packages import get_package_share_directory

            self.robot_properties_path = get_package_share_directory(
                "robot_properties_fingers"
            )
        except Exception:
            self.robot_properties_path = os.path.join(
                os.path.dirname(__file__), "robot_properties_fingers"
            )

        urdf_file = finger_types_data.get_finger_urdf(self.finger_type)
        self.finger_urdf_path = os.path.join(
            self.robot_properties_path, "urdf", urdf_file
        )

    def __load_robot_urdf(self, robot_position_offset):
        """
        Load the single/trifinger model from the corresponding urdf

        Args:
            robot_position_offset: Position offset with which the robot is
                placed in the world.  Use this, for example, to change the
                height of the fingers above the table.
        """
        finger_base_orientation = (0, 0, 0, 1)

        self.finger_id = pybullet.loadURDF(
            fileName=self.finger_urdf_path,
            basePosition=robot_position_offset,
            baseOrientation=finger_base_orientation,
            useFixedBase=1,
            flags=(
                pybullet.URDF_USE_INERTIA_FROM_FILE
                | pybullet.URDF_USE_SELF_COLLISION
            ),
            physicsClientId=self._pybullet_client_id,
        )

        # create a map link_name -> link_index
        # Source: https://pybullet.org/Bullet/phpBB3/viewtopic.php?t=12728.
        link_name_to_index = {
            pybullet.getBodyInfo(
                self.finger_id,
                physicsClientId=self._pybullet_client_id,
            )[0].decode("UTF-8"): -1,
        }
        for joint_idx in range(
            pybullet.getNumJoints(
                self.finger_id,
                physicsClientId=self._pybullet_client_id,
            )
        ):
            link_name = pybullet.getJointInfo(
                self.finger_id,
                joint_idx,
                physicsClientId=self._pybullet_client_id,
            )[12].decode("UTF-8")
            link_name_to_index[link_name] = joint_idx

        self.pybullet_link_indices = [
            link_name_to_index[name] for name in self.link_names
        ]
        self.pybullet_tip_link_indices = [
            link_name_to_index[name] for name in self.tip_link_names
        ]
        # joint and link indices are the same in pybullet
        self.pybullet_joint_indices = self.pybullet_link_indices

    def __load_stage(self, high_border=True):
        """Create the stage (table and boundary).

        Args:
            high_border:  Only used for the TriFinger.  If set to False, the
                old, low boundary will be loaded instead of the high one.
        """

        def mesh_path(filename):
            return os.path.join(self.robot_properties_path, "meshes", filename)

        if self.finger_type in ["fingerone", "fingeredu", "fingerpro"]:
            collision_objects.import_mesh(
                mesh_path("Stage_simplified.stl"),
                position=[0, 0, 0],
                is_concave=True,
                pybullet_client_id=self._pybullet_client_id,
            )

        elif self.finger_type == "trifingerone":
            table_colour = (0.18, 0.15, 0.19, 1.0)
            high_border_colour = (0.73, 0.68, 0.72, 1.0)
            if high_border:

                # use a simple cuboid for the table
                self._table_id = collision_objects.Cuboid(
                    position=(0, 0, -0.005),
                    orientation=(0, 0, 0, 1),
                    half_extents=(0.38, 0.38, 0.005),
                    mass=0,  # static object
                    color_rgba=table_colour,
                    pybullet_client_id=self._pybullet_client_id,
                )

                collision_objects.import_mesh(
                    mesh_path("object_containment_sim.stl"),
                    position=[0, 0, 0],
                    is_concave=True,
                    color_rgba=high_border_colour,
                    pybullet_client_id=self._pybullet_client_id,
                )
            else:
                collision_objects.import_mesh(
                    mesh_path("BL-M_Table_ASM_big.stl"),
                    position=[0, 0, 0],
                    is_concave=True,
                    color_rgba=table_colour,
                    pybullet_client_id=self._pybullet_client_id,
                )
        elif self.finger_type == "trifingerpro":
            table_colour = np.array((53.0, 58.0, 50.0, 255.0)) / 255.0
            high_border_colour = int_to_rgba(0x8F8D95)

            # use a simple cuboid for the table
            self._table_id = collision_objects.Cuboid(
                position=(0, 0, -0.005),
                orientation=(0, 0, 0, 1),
                half_extents=(0.38, 0.38, 0.005),
                mass=0,  # static object
                color_rgba=table_colour,
                pybullet_client_id=self._pybullet_client_id,
            )

            self._containment_id = collision_objects.import_mesh(
                mesh_path("object_containment_sim.stl"),
                position=[0, 0, 0],
                is_concave=True,
                color_rgba=high_border_colour,
                pybullet_client_id=self._pybullet_client_id,
            )

        elif self.finger_type == "trifingeredu":
            table_colour = (0.95, 0.95, 0.95, 1.0)
            high_border_colour = (0.95, 0.95, 0.95, 1.0)

            # use a simple cuboid for the table
            self._table_id = collision_objects.Cuboid(
                position=(0, 0, -0.005),
                orientation=(0, 0, 0, 1),
                half_extents=(0.38, 0.38, 0.005),
                mass=0,  # static object
                color_rgba=table_colour,
                pybullet_client_id=self._pybullet_client_id,
            )

            collision_objects.import_mesh(
                mesh_path("edu/frame_wall.stl"),
                position=[0, 0, 0],
                is_concave=True,
                color_rgba=high_border_colour,
                pybullet_client_id=self._pybullet_client_id,
            )
        else:
            raise ValueError("Invalid finger type '%s'" % self.finger_type)

    def __set_default_pd_gains(self):
        """Set the default PD gains depending on the finger type."""
        if self.finger_type in ["fingerpro", "trifingerpro"]:
            self.position_gains = np.array(
                [15.0, 15.0, 9.0] * self.number_of_fingers
            )
            self.velocity_gains = np.array(
                [0.5, 1.0, 0.5] * self.number_of_fingers
            )
        else:
            self.position_gains = np.array(
                [10.0, 10.0, 10.0] * self.number_of_fingers
            )
            self.velocity_gains = np.array(
                [0.1, 0.3, 0.001] * self.number_of_fingers
            )
