#!/usr/bin/env python3
import unittest

from trifinger_simulation.gym_wrapper.envs import cube_env


class TestSample(unittest.TestCase):
    """Test the CubeEnv gym environment."""

    def test_if_observations_are_valid(self):
        # Observations need to be contained in the observation space.  If this
        # is not the case, there is either an issue with the  generation of the
        # observations or the observation space is not defined properly.
        env = cube_env.CubeEnv(cube_env.RandomInitializer(4))

        observation = env.reset()
        self.assertTrue(
            env.observation_space.contains(observation),
            msg="observation: {}".format(observation),
        )

        for i in range(3000):
            action = env.action_space.sample()
            observation, _, _, _ = env.step(action)

            self.assertTrue(
                env.observation_space.contains(observation),
                msg="Invalid observation: {}".format(observation),
            )


if __name__ == "__main__":
    unittest.main()
