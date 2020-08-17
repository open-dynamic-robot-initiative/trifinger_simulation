#!/usr/bin/env python3
import unittest

from trifinger_simulation.gym_wrapper.envs import trifinger_cube_env


class TestSample(unittest.TestCase):
    """Test the TriFingerCubeEnv gym environment."""

    def test_if_observations_are_valid(self):
        # Observations need to be contained in the observation space.  If this
        # is not the case, there is either an issue with the  generation of the
        # observations or the observation space is not defined properly.
        env = trifinger_cube_env.TriFingerCubeEnv(trifinger_cube_env.RandomInitializer(4))

        observation = env.reset()
        self.assertTrue(env.observation_space.contains(observation))

        for i in range(3000):
            action = env.action_space.sample()
            observation, _, _, _ = env.step(action)

            self.assertTrue(env.observation_space.contains(observation))


if __name__ == "__main__":
    unittest.main()
