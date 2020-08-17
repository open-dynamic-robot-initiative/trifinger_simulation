#!/usr/bin/env python3
import argparse
import os
import gym
import numpy as np

from stable_baselines import PPO2
from trifinger_simulation.gym_wrapper.envs import trifinger_cube_env
from example_pushing_training_env import (
    ExamplePushingTrainingEnv,
    FlatObservationWrapper,
)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--model_path", required=True, help="model path")
    parser.add_argument("--time_steps", required=True, help="time steps")

    args = vars(parser.parse_args())
    time_steps = int(args["time_steps"])
    model_path = str(args["model_path"])

    policy_path = os.path.join(
        model_path, "model_" + str(time_steps) + "_steps"
    )

    model = PPO2.load(policy_path)

    # define a method for the policy fn of your trained model
    def policy_fn(obs):
        return model.predict(obs, deterministic=True)[0]

    # we create the same env as we used for training in train_pushing_ppo.py,
    # such that action and observation space remain coherent with the policy.
    # however, unlike during the training, we set the initialization to the the
    # same as in the standard TriFingerCubeEnv, since this is what the policy will be
    # evaluated on eventually.
    initializer = trifinger_cube_env.RandomInitializer(
        difficulty=1
    )  # difficulty one means pushing
    env = ExamplePushingTrainingEnv(
        initializer=initializer, frameskip=3, visualization=True
    )
    env = FlatObservationWrapper(env)

    for _ in range(10):
        obs = env.reset()
        done = False
        while not done:
            obs, rew, done, info = env.step(policy_fn(obs))
