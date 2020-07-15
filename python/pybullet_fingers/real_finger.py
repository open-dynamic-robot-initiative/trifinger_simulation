#!/usr/bin/env python3
# -------------------------------------------------------------------------------------------------
# The documentation in this code is heavily derived from the official
# documentation of PyBullet at
# https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#
# among other scattered sources.
# -------------------------------------------------------------------------------------------------
import pybullet
import pybullet_data

import robot_interfaces
import blmc_robots

from pybullet_fingers.base_finger import BaseFinger


class RealFinger(BaseFinger):
    """
    For interacting with the physical (real) finger and robot
    in the same way as with its simulated counterpart.
    """

    def __init__(
        self, finger_type, finger_config_suffix, enable_visualization=False,
    ):
        """
        Constructor, initializes the physical world we will work in.

        Args:
            finger_type: See BaseFinger.
            finger_config_suffix (int): ID of the finger that is used. Has to
                be one of [0, 120, 240]. This ignored if finger_type = "tri".
            enable_visualization (bool): Set to 'True' to run a GUI for
                visualization.  This uses pyBullet but only for visualization,
                i.e. the state of the simulation is constantly set to match the
                one of the real robot.
        """
        # Simulation is only used for visualization, so only run it when needed
        self.enable_simulation = enable_visualization

        super().__init__(finger_type, enable_visualization)

        if self.enable_simulation:
            self.make_physical_world()

        if finger_type == "tri":
            self.robot = blmc_robots.Robot(
                robot_interfaces.trifinger,
                blmc_robots.create_trifinger_backend,
                "trifinger.yml",
            )
        elif finger_type == "single":
            self.robot = blmc_robots.Robot(
                robot_interfaces.finger,
                blmc_robots.create_real_finger_backend,
                "finger_%s.yml" % finger_config_suffix,
            )
        else:
            raise ValueError("Invalid finger type")

        self.robot.initialize()
        self.Action = self.robot.Action

    def make_physical_world(self):
        """
        Set the physical parameters of the world in which the simulation
        will run, and import the models to be simulated
        """
        pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
        pybullet.setGravity(0, 0, -9.81)

        pybullet.loadURDF("plane_transparent.urdf", [0, 0, 0])
        self.import_finger_model()

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
        return self.robot.frontend.append_desired_action(action)

    def get_observation(self, time_index):
        """
        Get the observation from the robot at a specified time_index.

        Args:
            time_index (int): the time_index at which the observation is
                needed
        Returns:
            observation (robot.Observation): the corresponding observation
        """
        observation = self.robot.frontend.get_observation(time_index)

        if self.enable_simulation:
            for i, joint_id in enumerate(self.revolute_joint_ids):
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

    # dummy functions for compatible API
    # (refer to the SimFinger class for details)
    def set_real_time_sim(self, *args):
        pass
