#!/usr/bin/env python3
import unittest
import numpy as np

from trifinger_simulation.sim_finger import SimFinger


class TestRobotEquivalentInterface(unittest.TestCase):
    """
    Test the methods of SimFinger that provide an interface equivalent to the
    RobotFrontend of robot_interfaces.
    """

    def setUp(self):
        self.finger = SimFinger(
            finger_type="fingerone",
            time_step=0.001,
            enable_visualization=False,
        )

        self.start_position = [0, -0.7, -1.5]
        self.finger.reset_finger_positions_and_velocities(self.start_position)

    def tearDown(self):
        # destroy the simulation to ensure that the next test starts with a
        # clean state
        del self.finger

    def test_timing_action_t_vs_observation_t(self):
        """Verify that observation_t is really not affected by action_t."""
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
        np.testing.assert_array_equal(self.start_position, obs.position)

        # after applying another action (even with zero torque), we should see
        # the effect
        t = self.finger.append_desired_action(self.finger.Action())
        obs = self.finger.get_observation(t)
        # new position should be less, as negative torque is applied
        np.testing.assert_array_less(obs.position, self.start_position)

    def test_timing_action_t_vs_observation_tplus1(self):
        """Verify that observation_{t+1} is affected by action_t."""
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
        np.testing.assert_array_less(obs.position, self.start_position)

    def test_timing_observation_t_vs_tplus1(self):
        # Apply a max torque action for one step
        action = self.finger.Action(
            torque=[
                -self.finger.max_motor_torque,
                -self.finger.max_motor_torque,
                -self.finger.max_motor_torque,
            ]
        )
        t = self.finger.append_desired_action(action)
        obs_t = self.finger.get_observation(t)
        obs_tplus1 = self.finger.get_observation(t + 1)

        # newer position should be lesser, as negative torque is applied
        np.testing.assert_array_less(obs_tplus1.position, obs_t.position)

    def test_timing_observation_t_multiple_times(self):
        """
        Verify that calling get_observation(t) multiple times always gives same
        result.
        """
        # Apply a max torque action for one step
        action = self.finger.Action(
            torque=[
                -self.finger.max_motor_torque,
                -self.finger.max_motor_torque,
                -self.finger.max_motor_torque,
            ]
        )
        t = self.finger.append_desired_action(action)
        obs_t1 = self.finger.get_observation(t)

        # observation should not change when calling multiple times with same t
        for i in range(10):
            obs_ti = self.finger.get_observation(t)
            np.testing.assert_array_equal(obs_t1.position, obs_ti.position)
            np.testing.assert_array_equal(obs_t1.velocity, obs_ti.velocity)
            np.testing.assert_array_equal(obs_t1.torque, obs_ti.torque)
            np.testing.assert_array_equal(obs_t1.tip_force, obs_ti.tip_force)

    def test_timing_observation_tplus1_multiple_times(self):
        """
        Verify that calling get_observation(t + 1) multiple times always gives
        same result.
        """
        # Apply a max torque action for one step
        action = self.finger.Action(
            torque=[
                -self.finger.max_motor_torque,
                -self.finger.max_motor_torque,
                -self.finger.max_motor_torque,
            ]
        )
        t = self.finger.append_desired_action(action)
        obs_t1 = self.finger.get_observation(t + 1)

        # observation should not change when calling multiple times with same t
        for i in range(10):
            obs_ti = self.finger.get_observation(t + 1)
            np.testing.assert_array_equal(obs_t1.position, obs_ti.position)
            np.testing.assert_array_equal(obs_t1.velocity, obs_ti.velocity)
            np.testing.assert_array_equal(obs_t1.torque, obs_ti.torque)
            np.testing.assert_array_equal(obs_t1.tip_force, obs_ti.tip_force)

    def test_exception_on_old_t(self):
        """Verify that calling get_observation with invalid t raises error."""
        # verify that t < 0 is not accepted
        with self.assertRaises(ValueError):
            self.finger.get_observation(-1)

        # append two actions
        t1 = self.finger.append_desired_action(self.finger.Action())
        t2 = self.finger.append_desired_action(self.finger.Action())

        # it should be possible to get observation for t2 and t2 + 1 but not t1
        # or t2 + 2
        self.finger.get_observation(t2)
        self.finger.get_observation(t2 + 1)

        with self.assertRaises(ValueError):
            self.finger.get_observation(t1)

        with self.assertRaises(ValueError):
            self.finger.get_observation(t2 + 2)

    def test_observation_types(self):
        """Verify that all fields of observation are np.ndarrays."""
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

        # verify types
        self.assertIsInstance(obs.torque, np.ndarray)
        self.assertIsInstance(obs.position, np.ndarray)
        self.assertIsInstance(obs.velocity, np.ndarray)
        self.assertIsInstance(obs.tip_force, np.ndarray)

    def test_get_desired_action(self):
        # verify that t < 0 is not accepted
        with self.assertRaises(ValueError):
            self.finger.get_desired_action(-1)

        orig_action = self.finger.Action(
            torque=[1.0, 2.0, 3.0], position=[0.0, -1.0, -2.0]
        )
        t = self.finger.append_desired_action(orig_action)
        action = self.finger.get_desired_action(t)

        np.testing.assert_array_equal(orig_action.torque, action.torque)
        np.testing.assert_array_equal(orig_action.position, action.position)

        # verify types
        self.assertIsInstance(action.torque, np.ndarray)
        self.assertIsInstance(action.position, np.ndarray)
        self.assertIsInstance(action.position_kd, np.ndarray)
        self.assertIsInstance(action.position_kp, np.ndarray)

    def test_get_applied_action(self):
        # verify that t < 0 is not accepted
        with self.assertRaises(ValueError):
            self.finger.get_applied_action(-1)

        desired_action = self.finger.Action(torque=[5, 5, 5])
        t = self.finger.append_desired_action(desired_action)
        applied_action = self.finger.get_applied_action(t)

        np.testing.assert_array_almost_equal(
            applied_action.torque,
            [
                self.finger.max_motor_torque,
                self.finger.max_motor_torque,
                self.finger.max_motor_torque,
            ],
        )

        # verify types
        self.assertIsInstance(applied_action.torque, np.ndarray)
        self.assertIsInstance(applied_action.position, np.ndarray)
        self.assertIsInstance(applied_action.position_kd, np.ndarray)
        self.assertIsInstance(applied_action.position_kp, np.ndarray)

    def test_get_timestamp_ms_001(self):
        t = self.finger.append_desired_action(self.finger.Action())
        first_stamp = self.finger.get_timestamp_ms(t)
        t = self.finger.append_desired_action(self.finger.Action())
        second_stamp = self.finger.get_timestamp_ms(t)

        # time step is set to 0.001, so the difference between two steps should
        # be 1 ms.
        self.assertEqual(second_stamp - first_stamp, 1)

    def test_get_timestamp_ms_tplus1_001(self):
        t = self.finger.append_desired_action(self.finger.Action())
        stamp_t = self.finger.get_timestamp_ms(t)
        stamp_tp1 = self.finger.get_timestamp_ms(t + 1)

        # time step is set to 0.001, so the difference between two steps should
        # be 1 ms.
        self.assertEqual(stamp_tp1 - stamp_t, 1)

    def test_get_timestamp_ms_invalid_t(self):
        # verify that t < 0 is not accepted
        with self.assertRaises(ValueError):
            self.finger.get_timestamp_ms(-1)

        t = self.finger.append_desired_action(self.finger.Action())

        with self.assertRaises(ValueError):
            self.finger.get_timestamp_ms(t - 1)

        with self.assertRaises(ValueError):
            self.finger.get_timestamp_ms(t + 2)

    def test_get_current_timeindex(self):
        # no time index before first action
        with self.assertRaises(ValueError):
            self.finger.get_current_timeindex()

        t = self.finger.append_desired_action(self.finger.Action())
        self.assertEqual(self.finger.get_current_timeindex(), t)


if __name__ == "__main__":
    unittest.main()
