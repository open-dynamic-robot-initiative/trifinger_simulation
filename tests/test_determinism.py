#!/usr/bin/env python3
import unittest
import numpy as np

import gym

from trifinger_simulation.sim_finger import SimFinger


class TestSimulationDeterminisim(unittest.TestCase):
    """This test verifies that the simulation always behaves deterministically.

    When starting from the same position and sending the same commands, the
    result should be the same.
    """

    def test_constant_torque(self):
        """Compare two runs sending a constant torque.

        In each run the finger is reset to a fixed position and a constant
        torque is applied for a number of steps.  The final observation of each
        run is compared.  If the simulation behaves deterministically, the
        observations should be equal.
        """
        finger = SimFinger(
            finger_type="fingerone",
        )

        start_position = [0.5, -0.7, -1.5]
        action = finger.Action(torque=[0.3, 0.3, 0.3])

        def run():
            finger.reset_finger_positions_and_velocities(start_position)
            for i in range(30):
                finger._set_desired_action(action)
                finger._step_simulation()
            return finger._get_latest_observation()

        first_run = run()
        second_run = run()

        np.testing.assert_array_equal(first_run.torque, second_run.torque)
        np.testing.assert_array_equal(first_run.position, second_run.position)
        np.testing.assert_array_equal(first_run.velocity, second_run.velocity)

    def test_reach_rollouts(self):

        smoothing_params = {
            "num_episodes": 10,
            # ratio of total training time after which smoothing starts
            "start_after": 3.0 / 7.0,
            # smoothing coeff. that shall be reached at end of training
            "final_alpha": 0.975,
            "stop_after": 5.0 / 7.0,
        }

        num_samples = 10
        horizon = 100

        env = gym.make(
            "reach-v0",
            control_rate_s=0.02,
            enable_visualization=False,
            finger_type="fingerone",
            smoothing_params=smoothing_params,
        )

        start_position = [0.5, -0.7, -1.5]

        plans = -1.0 + 2.0 * np.random.rand(horizon, num_samples, 3)

        def run():
            states = []
            rewards = []

            for i in range(num_samples):
                env.reset()
                env.finger.reset_finger_positions_and_velocities(
                    start_position
                )
                for t in range(horizon):
                    state, reward, _, _ = env.step(plans[t, i])
                    states.append(state)
                    rewards.append(reward)
            states = np.array(states)
            rewards = np.array(rewards)
            return states, rewards

        first_states, first_rewards = run()
        second_states, second_rewards = run()

        np.testing.assert_array_equal(first_states.all(), second_states.all())
        np.testing.assert_array_equal(
            first_rewards.all(), second_rewards.all()
        )


if __name__ == "__main__":
    unittest.main()
