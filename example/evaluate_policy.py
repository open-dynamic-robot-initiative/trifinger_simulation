#!/usr/bin/env python3
"""Example evaluation script to evaluate a policy.

This is an example evaluation script which loads a policy trained with PPO. If
this script were moved into the top trifinger_simulation folder (since this is where
we will execute the rrc_evaluate command), it would consistute a valid
submission (naturally, imports below would have to be adjusted accordingly).

This script will be executed in an automated procedure.  For this to work, make
sure you do not change the overall structure of the script!

This script expects the following arguments in the given order:
 - Difficulty level (needed for reward computation)
 - initial pose of the cube (as JSON string)
 - goal pose of the cube (as JSON string)
 - file to which the action log is written

It is then expected to initialize the environment with the given initial pose
and execute exactly one episode with the policy that is to be evaluated.

When finished, the action log, which is created by the TriFingerPlatform class,
is written to the specified file.  This log file is crucial as it is used to
evaluate the actual performance of the policy.
"""
import sys
import os

import gym

from trifinger_simulation.gym_wrapper.envs import cube_env
from trifinger_simulation.tasks import move_cube

from example_pushing_training_env import ExamplePushingTrainingEnv
from example_pushing_training_env import FlatObservationWrapper

from stable_baselines import PPO2


class RandomPolicy:
    """Dummy policy which uses random actions."""

    def __init__(self, action_space):
        self.action_space = action_space

    def predict(self, observation):
        return self.action_space.sample()


class PPOPolicy:
    def __init__(self, path):
        self.ppo_policy = PPO2.load(path)

    def predict(self, observation):
        return self.ppo_policy.predict(observation, deterministic=True)[0]


def main():
    try:
        difficulty = int(sys.argv[1])
        initial_pose_json = sys.argv[2]
        goal_pose_json = sys.argv[3]
        output_file = sys.argv[4]
    except IndexError:
        print("Incorrect number of arguments.")
        print(
            "Usage:\n"
            "\tevaluate_policy.py <difficulty_level> <initial_pose>"
            " <goal_pose> <output_file>"
        )
        sys.exit(1)

    # the poses are passed as JSON strings, so they need to be converted first
    initial_pose = move_cube.Pose.from_json(initial_pose_json)
    goal_pose = move_cube.Pose.from_json(goal_pose_json)

    # create a FixedInitializer with the given values
    initializer = cube_env.FixedInitializer(
        difficulty, initial_pose, goal_pose
    )

    # if difficulty == 1 (i.e. pushing), we load the policy we trained for that
    # task. otherwise, we just use the RandomPolicy as placeholder. Naturally,
    # when you submit you would have a policy for each difficulty level.
    if difficulty == 1:

        # we create the same env as we used for training in
        # train_pushing_ppo.py, such that action and observation space remain
        # coherent with the policy. however, unlike during  training, we set the
        # initialization using the initializer, since this is what's expected
        # during evaluation. if you do not use the initializer, or modify the
        # standard CubeEnv in any way which will affect the simulation (i.e.
        # affect the state action trajectories), the action trajectories you
        # compute will not make sense.
        env = ExamplePushingTrainingEnv(
            initializer=initializer, frameskip=3, visualization=False
        )
        env = FlatObservationWrapper(env)

        # we load the trained policy
        policy_path = os.path.join(
            "./training_checkpoints", "model_78000000_steps"
        )
        policy = PPOPolicy(policy_path)

    else:
        env = gym.make(
            "trifinger_simulation.gym_wrapper:real_robot_challenge_phase_1-v1",
            initializer=initializer,
            action_type=cube_env.ActionType.POSITION,
            visualization=False,
        )
        policy = RandomPolicy(env.action_space)

    # Execute one episode.  Make sure that the number of simulation steps
    # matches with the episode length of the task.  When using the default Gym
    # environment, this is the case when looping until is_done == True.  Make
    # sure to adjust this in case your custom environment behaves differently!
    is_done = False
    observation = env.reset()
    accumulated_reward = 0
    while not is_done:
        action = policy.predict(observation)
        observation, reward, is_done, info = env.step(action)
        accumulated_reward += reward

    print("Accumulated reward: {}".format(accumulated_reward))

    # store the log for evaluation
    env.platform.store_action_log(output_file)


if __name__ == "__main__":
    main()
