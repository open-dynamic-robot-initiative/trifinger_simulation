#!/usr/bin/env python3
"""Evaluate a policy with the given goal.

Takes difficulty level (for reward computation), initial pose of the object and
goal pose of the object as arguments and uses them to test a dummy "random
policy".

The accumulated reward is computed and in the end the action log is written to
a file, for later verification.

You may use this as evaluation script for your final policy.  Simply replace
the `RandomPolicy` and potentially the Gym environment below (see the TODOs).
"""
import sys

import gym

from trifinger_simulation.gym_wrapper.envs import cube_env
from trifinger_simulation.tasks import move_cube


class RandomPolicy:
    """Dummy policy which uses random actions."""

    def __init__(self, action_space):
        self.action_space = action_space

    def predict(self, observation):
        return self.action_space.sample()


def main():
    # Difficulty level, initial pose and goal pose of the cube are passed as
    # arguments
    difficulty = int(sys.argv[1])
    initial_pose_json = sys.argv[2]
    goal_pose_json = sys.argv[3]

    # the poses are passes as JSON strings, so they need to be converted first
    initial_pose = move_cube.Pose.from_json(initial_pose_json)
    goal_pose = move_cube.Pose.from_json(goal_pose_json)

    # create a FixedInitializer with the given values
    initializer = cube_env.FixedInitializer(difficulty, initial_pose, goal_pose)

    # TODO: Replace with your environment if you used a custom one.
    env = gym.make(
        "trifinger_simulation.gym_wrapper:real_robot_challenge_phase_1-v1",
        initializer=initializer,
        action_type=cube_env.ActionType.POSITION,
        visualization=False,
    )

    # TODO: Replace this with your model
    policy = RandomPolicy(env.action_space)

    # execute one episode
    is_done = False
    observation = env.reset()
    accumulated_reward = 0
    while not is_done:
        action = policy.predict(observation)
        observation, reward, is_done, info = env.step(action)
        accumulated_reward += reward

    print("Accumulated reward: {}".format(accumulated_reward))

    # store the log for evaluation
    env.platform.store_action_log("action_log.json")


if __name__ == "__main__":
    main()
