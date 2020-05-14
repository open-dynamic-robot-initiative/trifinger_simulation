#!/usr/bin/env python3
import unittest
import numpy as np

import robot_interfaces
import pybullet_fingers.drivers


class TestPyBulletBackend(unittest.TestCase):
    """Test using pyBullet in the robot interface backend via Python."""

    def run_position_test(self, finger_type, goal_positions):
        """Run position test for single or tri-finger.

        Moves the robot in a sequence of goal positions in joint-space.  After
        each step, it is checked if the robot reached the goal with some
        tolerance.

        Args:
            finger_type:  Which robot to use.  One of ("single", "tri").
            goal_positions:  A list of joint goal positions.
        """
        # select the correct types/functions based on which robot is used
        if finger_type == "single":
            finger_types = robot_interfaces.finger
            create_backend = (
                pybullet_fingers.drivers.create_single_finger_backend
            )
        else:
            finger_types = robot_interfaces.trifinger
            create_backend = pybullet_fingers.drivers.create_trifinger_backend

        robot_data = finger_types.SingleProcessData()

        backend = create_backend(
            robot_data, real_time_mode=False, visualize=False
        )

        frontend = finger_types.Frontend(robot_data)
        backend.initialize()

        # Simple example application that moves the finger to random positions.
        for goal in goal_positions:
            action = finger_types.Action(position=goal)
            for _ in range(300):
                t = frontend.append_desired_action(action)
                frontend.wait_until_time_index(t)

            # check if desired position is reached
            current_position = frontend.get_observation(t).position
            np.testing.assert_array_almost_equal(
                goal, current_position, decimal=2
            )

    def test_single_finger_position_control(self):
        """Test position control for the simulated single finger."""
        goals = [
            [0.21, 0.32, -1.10],
            [0.69, 0.78, -1.07],
            [-0.31, 0.24, -0.20],
        ]
        self.run_position_test("single", goals)

    def test_trifinger_position_control(self):
        """Test position control for the simulated TriFinger."""
        goals = [
            [0.27, -0.72, -1.03, 0.53, -0.26, -1.67, -0.10, 0.10, -1.36],
            [0.38, -0.28, -1.91, 0.22, 0.02, -0.03, 0.00, 0.28, -0.03],
            [0.14, 0.27, -0.03, 0.08, -0.08, -0.03, 0.53, -0.00, -0.96],
        ]
        self.run_position_test("tri", goals)


if __name__ == "__main__":
    import rosunit
    rosunit.unitrun("pybullet_fingers", 'test_pybullet_backend',
                    TestPyBulletBackend)
