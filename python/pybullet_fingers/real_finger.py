#!/usr/bin/env python3
# -------------------------------------------------------------------------------------------------
# The documentation in this code is heavily derived from the official
# documentation of PyBullet at
# https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#
# among other scattered sources.
# -------------------------------------------------------------------------------------------------
import numpy as np

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
        self, enable_visualization, finger_type, finger_config_suffix,
    ):
        """
        Constructor, initializes the physical world we will work in.

        Args:
            enable_visualization (bool): Set to 'True' to run a GUI for
                visualization.  This uses pyBullet but only for visualization,
                i.e. the state of the simulation is constantly set to match the
                one of the real robot.
            finger_type: See BaseFinger.
            finger_config_suffix (int): ID of the finger that is used. Has to
                be one of [0, 120, 240]. This ignored if finger_type = "tri".
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

    def get_end_effector_position(self):
        """
        Get end effector position(s)

        Returns:
            Flat list of current end-effector positions, i.e. [x1, y1, z1, x2,
            y2, ...]
        """
        tip_positions = self.forward_kinematics(self.observation.position)
        return np.concatenate(tip_positions)

    def get_joint_positions_and_velocities(self):
        """
        Get the joint positions and velocities

        Returns:
            joint_positions (list of floats): Angular positions of all joints.
            joint_velocities (list of floats): Angular velocities of all joints
        """
        return self.observation.position, self.observation.velocity

    def make_physical_world(self):
        """
        Set the physical parameters of the world in which the simulation
        will run, and import the models to be simulated
        """
        pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
        pybullet.setGravity(0, 0, -9.81)

        pybullet.loadURDF("plane_transparent.urdf", [0, 0, 0])
        self.import_finger_model()

    def set_action(self, joint_positions, control_mode):
        """
        Specify the desired joint positions and specify "position" as the
        control mode to set the position field of the action with these

        Args:
            joint_positions (list of floats): The desired joint positions
            control_mode (string- "position"): The control mode. Currently only
                "position" is supported.

        Raises:
            NotImplementedError() if anything else from "position" is
            specified as the control_mode
        """
        if control_mode == "position":
            self.action = self.robot.Action(position=joint_positions)
        else:
            raise NotImplementedError()

    def step_robot(self, wait_for_observation):
        """
        Send action and wait for observation
        """
        t = self.robot.frontend.append_desired_action(self.action)

        if wait_for_observation:
            self.observation = self.robot.frontend.get_observation(t)

            if self.enable_visualization:
                for i, joint_id in enumerate(self.revolute_joint_ids):
                    pybullet.resetJointState(
                        self.finger_id, joint_id, self.observation.position[i]
                    )

    def reset_finger(self, joint_positions):
        """
        Move the finger(s) to a random position (sampled in the joint space).
        The sampled random position is set as target and the robot is stepped
        for one second to give it time to reach there.
        """
        self.action = self.robot.Action(position=joint_positions)
        for i in range(1000):
            self.step_robot(True)

        return joint_positions

    # dummy functions for compatible API
    # (refer to the SimFinger class for details)
    def set_real_time_sim(self, *args):
        pass
