#!/usr/bin/env python3
import unittest
import numpy as np

from pybullet_fingers.sim_finger import SimFinger


class TestActionObservationTiming(unittest.TestCase):
    """Test if time relation between action_t and observation_t are correct.
    """

    def setUp(self):
        self.finger = SimFinger(
            time_step=0.001, enable_visualization=False, finger_type="single",
        )

        start_position = [0, -0.7, -1.5]
        self.finger.reset_finger(start_position)

        # Run position control for several iterations to make sure the target
        # position is reached
        action = self.finger.Action(position=start_position)
        for i in range(1000):
            t = self.finger.append_desired_action(action)
            obs = self.finger.get_observation(t)

        self.initial_position = obs.position

    def test_timing_action_t_vs_observation_t(self):
        # Apply a max torque action for one step
        action = self.finger.Action(
            torque=[
                -self.finger.max_motor_torque,
                -self.finger.max_motor_torque,
                -self.finger.max_motor_torque,
            ]
        )
        t = self.finger.append_desired_action(action)
        obs = self.finger.get_observation(t)

        # as obs_t is from just before action_t is applied, the position should
        # not yet have changed
        np.testing.assert_array_equal(self.initial_position, obs.position)

        # after applying another action (even with zero torque), we should see
        # the effect
        t = self.finger.append_desired_action(self.finger.Action())
        obs = self.finger.get_observation(t)
        # new position should be less, as negative torque is applied
        np.testing.assert_array_less(obs.position, self.initial_position)

    def test_timing_action_t_vs_observation_tplus1(self):
        # Apply a max torque action for one step
        action = self.finger.Action(
            torque=[
                -self.finger.max_motor_torque,
                -self.finger.max_motor_torque,
                -self.finger.max_motor_torque,
            ]
        )
        t = self.finger.append_desired_action(action)
        obs = self.finger.get_observation(t + 1)

        # new position should be less, as negative torque is applied
        np.testing.assert_array_less(obs.position, self.initial_position)


if __name__ == "__main__":
    import rosunit

    rosunit.unitrun(
        "pybullet_fingers",
        "test_action_observation_timing",
        TestActionObservationTiming,
    )
