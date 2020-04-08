#!/usr/bin/env python3
# -------------------------------------------------------------------------------------------------
# The documentation in this code is heavily derived from the official
# documentation of PyBullet at
# https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#
# among other scattered sources.
# -------------------------------------------------------------------------------------------------
import copy
import os
import numpy as np

import pybullet
import pybullet_data

from pybullet_fingers.action import Action
from pybullet_fingers.observation import Observation
from pybullet_fingers.base_finger import BaseFinger
from pybullet_fingers import collision_objects


class SimFinger(BaseFinger):
    """
    A simulation environment for the single and the tri-finger robots.
    This environment is based on PyBullet, the official Python wrapper around
    the Bullet-C API.

    Attributes:

        position_gains (array): The kp gains for the pd control of the
            finger(s). Note, this depends on the simulation step size
            and has been set for a simulation rate of 250 Hz.
        velocity_gains (array):The kd gains for the pd control of the
            finger(s). Note, this depends on the simulation step size
            and has been set for a simulation rate of 250 Hz.
        safety_kd (array): The kd gains used for damping the joint motor
            velocities during the safety torque check on the joint motors.
        max_motor_torque (float): The maximum allowable torque that can
            be applied to each motor.
        action_index (int): An index used to enforce the structure of a
            time-series of length 1 for the action in which the application
            of the action precedes (in time) the observation corresponding
            to it. Incremented each time an action is applied.
        observation_index (int): The corresponding index for the observation
            to ensure the same structure as the action time-series for the
            observation time-series (of length 1).

    """

    def __init__(
        self, time_step, enable_visualization, finger_type,
    ):
        """
        Constructor, initializes the physical world we will work in.

        Args:
            time_step (float): It is the time between two simulation steps.
                Defaults to 1./240. Don't set this to be larger than 1./60.
                The gains etc are set according to a time_step of 0.004 s.
            enable_visualization (bool): See BaseFinger.
            finger_type: See BaseFinger.
        """
        # Always enable the simulation for the simulated robot :)
        self.enable_simulation = True

        super().__init__(finger_type, enable_visualization)

        self.time_step = time_step
        self.position_gains = np.array(
            [10.0, 10.0, 10.0] * self.number_of_fingers
        )
        self.velocity_gains = np.array(
            [0.1, 0.3, 0.001] * self.number_of_fingers
        )
        self.safety_kd = np.array([0.08, 0.08, 0.04] * self.number_of_fingers)
        self.max_motor_torque = 0.36

        self.action_index = -1
        self.observation_index = 0

        self.make_physical_world()
        self.disable_velocity_control()

    def Action(self, torque=None, position=None):
        """
        Fill in the fields of the action structure

        Args:
            torque (array): The torques to apply to the motors
            position (array): The absolute angular position to which
                the motors have to be rotated.

        Returns:
            the_action (Action): the action to be applied to the motors
        """
        if torque is None:
            torque = np.array([0.0] * 3 * self.number_of_fingers)
        if position is None:
            position = np.array([np.nan] * 3 * self.number_of_fingers)

        action = Action(torque, position)

        return action

    def set_real_time_sim(self, switch=0):
        """
        Choose to simulate in real-time or use a desired simulation rate
        Defaults to non-real time

        Args:
            switch (int, 0/1): 1 to set the simulation in real-time.
        """
        pybullet.setRealTimeSimulation(switch)

    def get_end_effector_position(self):
        """
        Get position(s) of the end-effector(s)

        Returns:
            Flat list of current end-effector positions, i.e. [x1, y1, z1, x2,
            y2, ...]
        """
        # TODO this should better return a list of lists/arrays instead of a
        # flat list
        end_effector_positions = []
        for finger_tip in self.finger_tip_ids:
            end_effector_positions += pybullet.getLinkState(
                self.finger_id, finger_tip
            )[0]

        return end_effector_positions

    def get_joint_positions_and_velocities(self):
        """
        Get the positions and velocities of the joints

        Returns:
            joint_positions (list of floats): Angular positions of all joints.
            joint_velocities (list of floats): Angular velocities of all joints
        """
        joints_data = pybullet.getJointStates(
            self.finger_id, self.revolute_joint_ids
        )
        positions = [joint[0] for joint in joints_data]
        velocities = [joint[1] for joint in joints_data]

        return positions, velocities

    def make_physical_world(self):
        """
        Set the physical parameters of the world in which the simulation
        will run, and import the models to be simulated
        """
        pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
        pybullet.setGravity(0, 0, -9.81)
        pybullet.setTimeStep(self.time_step)

        pybullet.loadURDF("plane_transparent.urdf", [0, 0, 0])
        self.import_finger_model()
        self.set_dynamics_properties()
        self.create_stage()

    def set_dynamics_properties(self):
        """
        To change properties of the robot such as its mass, friction, damping,
        maximum joint velocities etc.
        """
        for link_id in self.finger_link_ids:
            pybullet.changeDynamics(
                bodyUniqueId=self.finger_id,
                linkIndex=link_id,
                maxJointVelocity=1e3,
                restitution=0.8,
                jointDamping=0.0,
                lateralFriction=0.1,
                spinningFriction=0.1,
                rollingFriction=0.1,
                linearDamping=0.5,
                angularDamping=0.5,
                contactStiffness=0.1,
                contactDamping=0.05,
            )

    def create_stage(self, high_border=True):
        """Create the stage (table and boundary).

        Args:
            high_border:  Only used for the TriFinger.  If set to False, the
                old, low boundary will be loaded instead of the high one.
        """

        def mesh_path(filename):
            return os.path.join(
                self.robot_properties_path, "meshes", "stl", filename
            )

        if "single" in self.finger_type:
            collision_objects.import_mesh(
                mesh_path("Stage_simplified.stl"),
                position=[0, 0, 0.01],
                is_concave=True,
            )

        elif "tri" in self.finger_type:
            table_colour = (0.31, 0.27, 0.25, 1.0)
            high_border_colour = (0.95, 0.95, 0.95, 1.0)
            if high_border:
                collision_objects.import_mesh(
                    mesh_path("trifinger_table_without_border.stl"),
                    position=[0, 0, 0.01],
                    is_concave=False,
                    color_rgba=table_colour,
                )
                collision_objects.import_mesh(
                    mesh_path("high_table_boundary.stl"),
                    position=[0, 0, 0.01],
                    is_concave=True,
                    color_rgba=high_border_colour,
                )
            else:
                collision_objects.import_mesh(
                    mesh_path("BL-M_Table_ASM_big.stl"),
                    position=[0, 0, 0.01],
                    is_concave=True,
                    color_rgba=table_colour,
                )
        else:
            raise ValueError("Invalid finger type '%s'" % self.finger_type)

    def print_link_information(self):
        """
        Print the link indices along with their names as defined in the URDF.
        """
        for link_name, link_index in self.link_name_to_index.items():
            print(link_index, link_name)

    def print_joint_information(self):
        """
        Print the joint indices along with their names as defined in the URDF.
        """

        for joint_index in range(pybullet.getNumJoints(self.finger_id)):
            joint_name = pybullet.getJointInfo(self.finger_id, joint_index)[
                1
            ].decode("UTF-8")
            print(joint_index, joint_name)

    def disable_velocity_control(self):
        """
        To disable the high friction velocity motors created by
        default at all revolute and prismatic joints while loading them from
        the urdf.
        """

        pybullet.setJointMotorControlArray(
            bodyUniqueId=self.finger_id,
            jointIndices=self.revolute_joint_ids,
            controlMode=pybullet.VELOCITY_CONTROL,
            targetVelocities=[0] * len(self.revolute_joint_ids),
            forces=[0] * len(self.revolute_joint_ids),
        )

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
                self.compute_pd_control_torques(
                    desired_action.position,
                    applied_action.position_kp,
                    applied_action.position_kd,
                )
            )

        applied_action.torque = self._set_motor_torques(
            torque_command.tolist()
        )

        return applied_action

    def append_desired_action(self, action):
        """
        Pass an action on which safety checks
        will be performed and then the action will be applied to the motors.

        Args:
            action (Action): Joint positions or torques or both

        Returns:
            self.action_index (int): The current time-index at which the action
                was applied.
        """
        if self.action_index >= self.observation_index:
            raise Exception(
                "You have to call get_observation after each"
                "append_desired_action."
            )

        self._set_desired_action(action)

        self.action_index = self.action_index + 1
        return self.action_index

    def _get_latest_observation(self):
        """Get observation of the current state.

        Returns:
            observation (Observation): the joint positions, velocities, and
                torques of the joints.
        """
        observation = Observation()
        current_joint_states = pybullet.getJointStates(
            self.finger_id, self.revolute_joint_ids
        )

        observation.position = np.array(
            [joint[0] for joint in current_joint_states]
        )
        observation.velocity = np.array(
            [joint[1] for joint in current_joint_states]
        )
        observation.torque = np.array(
            [joint[3] for joint in current_joint_states]
        )

        return observation

    def get_observation(self, time_index):
        """
        Get the observation at the time of
        applying the action, so the observation actually corresponds
        to the state of the environment due to the application of the
        previous action.

        This method steps the simulation!

        Args:
            time_index (int): the time index at which the observation is
                needed. This can only be the current time-index (so same as the
                action_index)

        Returns:
            observation (Observation): the joint positions, velocities, and
                torques of the joints.

        Raises:
            Exception if the observation at any other time index than the one
            at which the action is applied, is queried for.
        """
        if not time_index == self.action_index:
            raise Exception(
                "currently you can only get the latest" "observation"
            )

        assert (
            self.observation_index == self.action_index
        ), "observation_index {} != action_index {}".format(
            self.observation_index, self.action_index
        )

        observation = self._get_latest_observation()
        self.observation_index = self.observation_index + 1

        self._step_simulation()

        return observation

    def compute_pd_control_torques(self, joint_positions, kp=None, kd=None):
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
            self.finger_id, self.revolute_joint_ids
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

    def pybullet_inverse_kinematics(self, desired_tip_positions):
        """
        Compute the joint angular positions needed to get to reach the block.

        WARNING: pybullet's inverse kinematics seem to be very inaccurate! (or
        we are somehow using it wrongly...)

        Args:
            finger (SimFinger): a SimFinger object
            desired_tip_position (list of floats): xyz target position for
                each finger tip.

        Returns:
            joint_pos (list of floats): The angular positions to be applid at
                the joints to reach the desired_tip_position
        """
        at_target_threshold = 0.0001

        # joint_pos = list(
        #    pybullet.calculateInverseKinematics2(
        #        bodyUniqueId=self.finger_id,
        #        endEffectorLinkIndices=self.finger_tip_ids,
        #        targetPositions=desired_tip_positions,
        #        residualThreshold=at_target_threshold))

        # For some reason calculateInverseKinematics2 above is not working
        # properly (only gives proper results for the joints of the first
        # finger).  As a workaround for now, call calculateInverseKinematics
        # for a single end-effector for each finger and always pick only the
        # joint positions for that finger.

        joint_pos = [None] * len(self.revolute_joint_ids)
        for i, (tip_index, desired_tip_position) in enumerate(
            zip(self.finger_tip_ids, desired_tip_positions)
        ):

            q = list(
                pybullet.calculateInverseKinematics(
                    bodyUniqueId=self.finger_id,
                    endEffectorLinkIndex=tip_index,
                    targetPosition=desired_tip_position,
                    residualThreshold=at_target_threshold,
                    maxNumIterations=100000,
                )
            )
            range_start = i * 3
            range_end = range_start + 3
            joint_pos[range_start:range_end] = q[range_start:range_end]

        return joint_pos

    def _set_motor_torques(self, desired_torque_commands):
        """
        Send torque commands to the motors.

        Args:
            desired_torque_commands (list of floats): The desired torques to be
                applied to the motors.  The torques that are actually applied
                may differ as some safety checks are applied.  See return
                value.

        Returns:
            List of torques that is actually set after applying safety checks.

        """
        torque_commands = self._safety_torque_check(desired_torque_commands)

        pybullet.setJointMotorControlArray(
            bodyUniqueId=self.finger_id,
            jointIndices=self.revolute_joint_ids,
            controlMode=pybullet.TORQUE_CONTROL,
            forces=torque_commands,
        )

        return torque_commands

    def _safety_torque_check(self, desired_torques):
        """
        Perform a check on the torques being sent to be applied to
        the motors so that they do not exceed the safety torque limit

        Args:
            desired_torques (list of floats): The torques desired to be
                applied to the motors

        Returns:
            applied_torques (list of floats): The torques that can be actually
                applied to the motors (and will be applied)
        """
        applied_torques = np.clip(
            np.asarray(desired_torques),
            -self.max_motor_torque,
            +self.max_motor_torque,
        )

        current_joint_states = pybullet.getJointStates(
            self.finger_id, self.revolute_joint_ids
        )
        current_velocity = np.array(
            [joint[1] for joint in current_joint_states]
        )
        applied_torques -= self.safety_kd * current_velocity

        applied_torques = list(
            np.clip(
                np.asarray(applied_torques),
                -self.max_motor_torque,
                +self.max_motor_torque,
            )
        )

        return applied_torques

    def _step_simulation(self):
        """
        Step the simulation to go to the next world state.
        """
        pybullet.stepSimulation()

    def set_action(self, joint_positions, control_mode):
        """
        Specify the desired joint positions and specify "position"
            as the control mode to set the position field of the
            action with these

        Args:
            joint_positions (list of floats): The desired joint positions
            control_mode (string- "position): The field of the action
                structure to be set

        Raises:
            NotImplementedError() if anything else from "position"
            is specified as the control_mode
        """
        if control_mode == "position":
            self.action = self.Action(position=joint_positions)
        else:
            raise NotImplementedError()

    def step_robot(self, wait_for_observation):
        """
        Send action and wait for observation
        """
        t = self.append_desired_action(self.action)
        observation = self.get_observation(t)
        if wait_for_observation:
            self.observation = observation

    def reset_finger(self, joint_positions):
        """
        Reset the finger(s) to some random position (sampled in the joint
        space) and step the robot with this random position

        Args:
            joint_positions (array-like):  Angular position for each joint.  If
                None, a random position is sampled.
        """
        for i, joint_id in enumerate(self.revolute_joint_ids):
            pybullet.resetJointState(
                self.finger_id, joint_id, joint_positions[i]
            )

        self.action = self.Action(position=joint_positions)
        self.step_robot(True)

        return joint_positions
