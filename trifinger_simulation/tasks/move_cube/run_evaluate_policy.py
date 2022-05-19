#!/usr/bin/env python3
"""Run `evaluate_policy.py` with random goals for all difficulty levels

Creates a dataset of multiple pairs of random initial state and goal for the
cube for each difficulty level.  Then executes `evaluate_policy.py` on each of
these samples and collects the log files in the specified output directory.

The `evaluate_policy.py` script is expected to be located in the current
working directory.  Replace the dummy policy there with your actual policy to
evaluate it.

This is used for the evaluation of the submissions of this phase of the first
phase of the real robot challenge.  Make sure this script runs with your
`evaluate_policy.py` script without any errors before submitting your code.
"""

# IMPORTANT:  DO NOT MODIFY THIS FILE!
# Submissions will be evaluate on our side with a similar script but not
# exactly this one.  To make sure that your code is compatible with our
# evaluation script, make sure it runs with this one without any modifications.

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
    levels: typing.Sequence[int], samples_per_level: int, logfile_tmpl: str
) -> typing.List[TestSample]:
    """Generate random test set for policy evaluation.

    For each difficulty level a list of samples, consisting of randomized
    initial pose and goal pose, is generated.

    Args:
        levels:  List of difficulty levels.
        samples_per_level:  How many samples are generated per level.
        logfile_tmpl:  Format string for the log file associated with this
            sample.  Needs to contain placeholders "{level}" and "{iteration}".

    Returns:
        List of ``len(levels) * samples_per_level`` test samples.
    """
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
    """Run evaluate_policy.py with the given sample.

    Args:
        sample (TestSample): Contains all required information to run the
            evaluation script.
    """
    cmd = [
        "python3",
        "./evaluate_policy.py",  # TODO: make path configurable?
        str(sample.difficulty),
        sample.init_pose_json,
        sample.goal_pose_json,
        sample.logfile,
    ]
    subprocess.run(cmd, check=True)


def main(output_directory: str):

    if not os.path.isdir(output_directory):
        print(
            "'{}' does not exist or is not a directory.".format(
                output_directory
            )
        )
        sys.exit(1)

    levels = (1, 2, 3, 4)
    runs_per_level = 10

    logfile_tmpl = os.path.join(
        output_directory, "action_log_l{level}_i{iteration}.p"
    )

    # generate n samples for each level
    test_data = generate_test_set(levels, runs_per_level, logfile_tmpl)

    # store samples
    sample_file = os.path.join(output_directory, "test_data.p")
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


def add_arguments(parser):
    parser.add_argument(
        "output_directory",
        type=str,
        help="Directory in which generated files are stored.",
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    add_arguments(parser)
    args = parser.parse_args()
    main(args.output_directory)
