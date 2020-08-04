#!/usr/bin/env python3
"""Demo on how to run the simulation using the Gym environment

This demo creates a CubeEnv environment and runs one episode with random
initialization using a dummy policy which uses random actions.
"""
import gym

from trifinger_simulation.gym_wrapper.envs import cube_env


class RandomPolicy:
    """Dummy policy which uses random actions."""

    def __init__(self, action_space):
        self.action_space = action_space

    def predict(self, observation):
        return self.action_space.sample()


def main():
    # Use a random initializer with difficulty 1
    initializer = cube_env.RandomInitializer(difficulty=1)

    env = gym.make(
        "trifinger_simulation.gym_wrapper:real_robot_challenge_phase_1-v1",
        initializer=initializer,
        action_type=cube_env.ActionType.POSITION,
        frameskip=100,
        visualization=True,
    )

    policy = RandomPolicy(env.action_space)

    observation = env.reset()
    is_done = False
    while not is_done:
        action = policy.predict(observation)
        observation, reward, is_done, info = env.step(action)

    print("Reward at final step: {:.3f}".format(reward))


if __name__ == "__main__":
    main()
