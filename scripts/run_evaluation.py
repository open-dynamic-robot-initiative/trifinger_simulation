#!/usr/bin/env python3
import argparse
import os
import pickle
import subprocess
import sys
import typing

import numpy as np

from trifinger_simulation.tasks import move_cube


class TestSample(typing.NamedTuple):
    difficulty: int
    iteration: int
    init_pose_json: str
    goal_pose_json: str
    logfile: str


def generate_test_set(
    levels: list, samples_per_level: int, logfile_tmpl: str
) -> list:
    samples = []
    for level in levels:
        for i in range(samples_per_level):
            init = move_cube.sample_goal(-1)
            goal = move_cube.sample_goal(level)
            logfile = logfile_tmpl.format(level=level, iteration=i)

            samples.append(
                TestSample(level, i, init.to_json(), goal.to_json(), logfile)
            )

    return samples


def run_evaluate_policy(sample: TestSample):
    cmd = [
        "./evaluate_policy.py",  # TODO: make path configurable?
        str(sample.difficulty),
        sample.init_pose_json,
        sample.goal_pose_json,
        sample.logfile,
    ]
    subprocess.run(cmd, check=True)


def run_replay(sample: TestSample) -> float:
    cmd = [
        "./replay_action_log.py",
        "--difficulty",
        str(sample.difficulty),
        "--initial-pose",
        sample.init_pose_json,
        "--goal-pose",
        sample.goal_pose_json,
        "--logfile",
        sample.logfile,
    ]
    res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    if res.returncode != 0:
        stderr = res.stderr.decode("utf-8")
        raise RuntimeError(
            "Replay of {} failed.  Output: {}".format(sample.logfile, stderr)
        )

    # extract the reward from the output
    output = res.stdout.decode("utf-8").split("\n")
    label = "Accumulated Reward: "
    reward = None
    for line in output:
        if line.startswith(label):
            reward = float(line[len(label) :])
            break

    if reward is None:
        raise RuntimeError("Failed to parse reward from relay.")

    return reward


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-directory",
        "-d",
        type=str,
        required=True,
        help="Directory in which generated files are stored.",
    )
    args = parser.parse_args()

    if not os.path.isdir(args.output_directory):
        print(
            "'{}' does not exist or is not a directory.".format(
                args.output_directory
            )
        )
        sys.exit(1)

    levels = (1, 2, 3, 4)
    runs_per_level = 10

    logfile_tmpl = os.path.join(
        args.output_directory, "action_log_l{level}_i{iteration}.json"
    )

    # generate n samples for each level
    test_data = generate_test_set(levels, runs_per_level, logfile_tmpl)

    # store samples
    sample_file = os.path.join(args.output_directory, "test_data.p")
    with open(sample_file, "wb") as fh:
        pickle.dump(test_data, fh)

    # run "evaluate_policy.py" for each sample
    for sample in test_data:
        print(
            "\n___Evaluate level {} sample {}___".format(
                sample.difficulty, sample.iteration
            )
        )
        run_evaluate_policy(sample)

    # run "replay_action_log.py" for each sample
    level_rewards = {level: [] for level in levels}
    for sample in test_data:
        print(
            "\n___Replay level {} sample {}___".format(
                sample.difficulty, sample.iteration
            )
        )
        level_rewards[sample.difficulty].append(run_replay(sample))

    # report
    print("\n==========================================================\n")
    total_reward = 0
    for level, rewards in level_rewards.items():
        rewards = np.asarray(rewards)
        mean = rewards.mean()
        print(
            "Level {} mean reward:\t{:.3f},\tstd: {:.3f}".format(
                level, mean, rewards.std()
            )
        )
        total_reward += level * mean

    print("----------------------------------------------------------")
    print("Total Weighted Reward: {:.3f}".format(total_reward))


if __name__ == "__main__":
    main()
