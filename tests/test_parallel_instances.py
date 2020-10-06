#!/usr/bin/env python3
"""Test two parallel simulation instances."""
import unittest
import numpy as np

from trifinger_simulation.sim_finger import SimFinger


class TestParallelInstances(unittest.TestCase):
    def test_step_one(self):
        # Create two instances, send actions and step only one.  Verify that
        # the other one does not move.
        robot1 = SimFinger(finger_type="trifingerpro")
        robot2 = SimFinger(finger_type="trifingerpro")

        start_position = np.array([0.0, 0.7, -1.5] * 3)

        robot1.reset_finger_positions_and_velocities(start_position)
        robot2.reset_finger_positions_and_velocities(start_position)

        action = robot1.Action(torque=[0.3, 0.3, 0.3] * 3)

        for i in range(30):
            robot1._set_desired_action(action)
            robot1._step_simulation()
            obs1 = robot1._get_latest_observation()
            obs2 = robot2._get_latest_observation()

            self.assertTrue((start_position != obs1.position).any())
            np.testing.assert_array_equal(start_position, obs2.position)

    def test_step_both(self):
        # Create two instances and send different actions to them.
        # Verify that both go towards their target
        robot1 = SimFinger(finger_type="trifingerpro")
        robot2 = SimFinger(finger_type="trifingerpro")

        start_position = np.array([0.0, 0.7, -1.5] * 3)

        robot1.reset_finger_positions_and_velocities(start_position)
        robot2.reset_finger_positions_and_velocities(start_position)

        action1 = robot1.Action(position=[0.5, 0.7, -1.5] * 3)
        action2 = robot2.Action(position=[-0.1, 0.7, -1.5] * 3)

        for i in range(1000):
            t1 = robot1.append_desired_action(action1)
            t2 = robot2.append_desired_action(action2)
            obs1 = robot1.get_observation(t1)
            obs2 = robot2.get_observation(t2)

            if i > 1:
                self.assertTrue((obs2.position != obs1.position).any())

        self.assertLess(np.linalg.norm(action1.position - obs1.position), 0.1)
        self.assertLess(np.linalg.norm(action2.position - obs2.position), 0.1)


if __name__ == "__main__":
    unittest.main()
