#!/usr/bin/env python3
import unittest
import numpy as np

from pybullet_fingers.sim_finger import SimFinger


class TestActionObservationTiming(unittest.TestCase):
    """Test if time relation between action_t and observation_t are correct.
    """

    def test_timing_action_t_vs_observation_t(self):
        finger = SimFinger(
            time_step=0.001, enable_visualization=False, finger_type="single",
        )

        target_position = [0, -0.7, -1.5]
        finger.reset_finger(target_position)

        # Run position control for several iterations to make sure the target
        # position is reached
        action = finger.Action(position=target_position)
        for i in range(1000):
            t = finger.append_desired_action(action)
            obs = finger.get_observation(t)

        initial_position = obs.position

        # Apply a max torque action for one step
        action = finger.Action(
            torque=[
                -finger.max_motor_torque,
                -finger.max_motor_torque,
                -finger.max_motor_torque,
            ]
        )
        t = finger.append_desired_action(action)
        obs = finger.get_observation(t)

        # as obs_t is from just before action_t is applied, the position should
        # not yet have changed
        np.testing.assert_array_equal(initial_position, obs.position)

        # after applying another action (even with zero torque), we should see
        # the effect
        t = finger.append_desired_action(finger.Action())
        obs = finger.get_observation(t)
        # new position should be less, as negative torque is applied
        np.testing.assert_array_less(obs.position, initial_position)


if __name__ == "__main__":
    import rosunit

    rosunit.unitrun(
        "pybullet_fingers",
        "test_action_observation_timing",
        TestActionObservationTiming,
    )
