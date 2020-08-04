#!/usr/bin/env python3
# -------------------------------------------------------------------------------------------------
# The documentation in this code is heavily derived from the official
# documentation of PyBullet at
# https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#
# among other scattered sources.
# -------------------------------------------------------------------------------------------------
import os
import rospkg

import pybullet
import pybullet_data

import robot_interfaces
import robot_fingers
from trifinger_simulation.base_finger import BaseFinger
from trifinger_simulation import finger_types_data


class RealFinger(BaseFinger):
    """
    The RealFinger class provides an interface to the real robot. Any script
    that creates an instance of the :class:`.SimFinger` can create an instance
    of this class and use it in the same way.
    """

    def __init__(
        self, finger_type, finger_config_suffix, enable_visualization=False,
    ):
        """
        Constructor, initializes the physical world we will work in.

        Args:
            finger_type (string): Name of the finger type.  In order to get
                a dictionary of the valid finger types, call
                :meth:`get_valid_finger_types` from finger_types_data
            finger_config_suffix (int): ID of the finger that is used. Has to
                be one of [0, 120, 240]. This is only if a single finger is to
                be used on any of the robots, and is ignored otherwise.
            enable_visualization (bool, optional): Set to 'True' to run a GUI
                for visualization.  This uses pyBullet but only for
                visualization, i.e. the state of the simulation is constantly
                set to match the one of the real robot.
        """
        # Simulation is only used for visualization, so only run it when needed
        self.enable_simulation = enable_visualization

        super().__init__(finger_type, enable_visualization)

        if self.enable_simulation:
            self.__setup_pybullet()

        number_of_fingers = finger_types_data.get_number_of_fingers(
            finger_type
        )

        if number_of_fingers == 1:
            if finger_type in ["single", "fingerone"]:
                config_file_path = os.path.join(
                    rospkg.RosPack().get_path("robot_fingers"),
                    "config",
                    "finger_%s.yml" % finger_config_suffix,
                )
            elif finger_type == "fingeredu":
                config_file_path = os.path.join(
                    rospkg.RosPack().get_path("robot_fingers"),
                    "config",
                    "fingeredu_%s.yml" % finger_config_suffix,
                )
            finger_data = robot_interfaces.finger.SingleProcessData()
            self.real_finger_backend = robot_fingers.create_real_finger_backend(
                finger_data, config_file_path
            )
            self.robot = robot_interfaces.finger.Frontend(finger_data)
            self.Action = robot_interfaces.finger.Action
        elif number_of_fingers == 3:
            if finger_type in ["tri", "trifingerone"]:
                config_file_path = os.path.join(
                    rospkg.RosPack().get_path("robot_fingers"),
                    "config",
                    "trifinger.yml",
                )
            elif finger_type == "trifingeredu":
                config_file_path = os.path.join(
                    rospkg.RosPack().get_path("robot_fingers"),
                    "config",
                    "trifingeredu.yml",
                )
            finger_data = robot_interfaces.trifinger.SingleProcessData()
            self.real_finger_backend = robot_fingers.create_trifinger_backend(
                finger_data, config_file_path
            )
            self.robot = robot_interfaces.trifinger.Frontend(finger_data)
            self.Action = robot_interfaces.trifinger.Action

        self.real_finger_backend.initialize()

    def __setup_pybullet(self):
        """
        Set the physical parameters of the world in which the simulation
        will run, and import the models to be simulated.
        """
        pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
        pybullet.setGravity(0, 0, -9.81)

        pybullet.loadURDF("plane_transparent.urdf", [0, 0, 0])
        self._load_robot_urdf()

    def append_desired_action(self, action):
        """
        Append an action to the action timeseries, that should be
        applied to the robot.

        Args:
            action (self.Action): Joint positions or torques or both

        Returns:
            self.action_index (int): The current time-index at which the action
                was applied.
        """
        return self.robot.append_desired_action(action)

    def get_observation(self, time_index):
        """
        Get the observation from the robot at a specified time_index.

        Args:
            time_index (int): the time_index at which the observation is
                needed
        Returns:
            observation (robot.Observation): the corresponding observation
        """
        observation = self.robot.get_observation(time_index)

        if self.enable_simulation:
            for i, joint_id in enumerate(self.pybullet_joint_indices):
                pybullet.resetJointState(
                    self.finger_id, joint_id, observation.position[i]
                )

        return observation

    def reset_finger(self, joint_positions):
        """
        Move the finger(s) to a random position (sampled in the joint space).
        The sampled random position is set as target and the robot is stepped
        for one second to give it time to reach there.
        """
        action = self.Action(position=joint_positions)
        for i in range(1000):
            t = self.append_desired_action(action)
            observation = self.get_observation(t)
        return observation

