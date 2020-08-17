#!/usr/bin/env python3
import argparse
import os
import gym
import numpy as np
from datetime import datetime

import tensorflow as tf

tf.compat.v1.logging.set_verbosity(tf.compat.v1.logging.ERROR)

from stable_baselines.common.vec_env import SubprocVecEnv
from stable_baselines import PPO2
from stable_baselines.common.policies import MlpPolicy
from stable_baselines.common import set_global_seeds
from stable_baselines.common.callbacks import CheckpointCallback

from example_pushing_training_env import (
    ExamplePushingTrainingEnv,
    FlatObservationWrapper,
)


def get_multi_process_env(num_of_envs):
    def _make_env(rank):
        def _init():
            env = ExamplePushingTrainingEnv(frameskip=3, visualization=False)
            env.seed(seed=rank)
            env.action_space.seed(seed=rank)
            env = FlatObservationWrapper(env)
            return env

        return _init

    return SubprocVecEnv([_make_env(rank=i) for i in range(num_of_envs)])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output-path",
        "-o",
        required=True,
        help="Specify the path to the directory where you want the model"
        " and the tensorboard log file to be stored.",
    )
    parser.add_argument(
        "--save-frequency",
        "-s",
        help="Specify how frequently you want to save the model.",
    )
    args = vars(parser.parse_args())

    output_dir = os.path.join(
        args["output_path"],
        datetime.now().strftime("%Y%m%d-%H%M%S-" + "ppopush"),
    )
    if not os.path.isdir(output_dir):
        os.makedirs(output_dir)
    log_path = os.path.join(output_dir, "logs")
    model_path = os.path.join(output_dir, "model")

    total_time_steps = 80000000
    validate_every_timesteps = 2000000

    set_global_seeds(0)
    num_of_active_envs = 20
    policy_kwargs = dict(layers=[256, 256])
    env = get_multi_process_env(num_of_active_envs)

    train_configs = {
        "gamma": 0.99,
        "n_steps": int(120000 / 20),
        "ent_coef": 0.01,
        "learning_rate": 0.00025,
        "vf_coef": 0.5,
        "max_grad_norm": 0.5,
        "nminibatches": 40,
        "noptepochs": 4,
    }

    model = PPO2(
        MlpPolicy,
        env,
        _init_setup_model=True,
        policy_kwargs=policy_kwargs,
        **train_configs,
        verbose=1,
        tensorboard_log=log_path
    )

    if args["save_frequency"] is None:
        save_frequency = int(validate_every_timesteps / num_of_active_envs)
    else:
        save_frequency = int(args["save_frequency"])
    checkpoint_callback = CheckpointCallback(
        save_freq=save_frequency, save_path=model_path, name_prefix="model"
    )

    print("~~~~~~~~~~~~ Training Started ~~~~~~~~~~~~")
    print(
        "Model and tensorboard logs to be stored inside {}".format(output_dir)
    )
    print("Model to be saved every {} steps".format(save_frequency))

    model.learn(int(total_time_steps), callback=checkpoint_callback)
    env.close()

    print("~~~~~~~~~~~~ Training Over ~~~~~~~~~~~~")
    print("\n\n\n")


if __name__ == "__main__":
    main()
