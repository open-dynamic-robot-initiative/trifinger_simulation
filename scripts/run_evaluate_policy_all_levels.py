#!/usr/bin/env python3
import argparse
import os
import pickle
import subprocess
import sys
import typing

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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "output_directory",
        type=str,
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
        pickle.dump(test_data, fh, pickle.HIGHEST_PROTOCOL)

    # run "evaluate_policy.py" for each sample
    for sample in test_data:
        print(
            "\n___Evaluate level {} sample {}___".format(
                sample.difficulty, sample.iteration
            )
        )
        run_evaluate_policy(sample)


if __name__ == "__main__":
    main()
