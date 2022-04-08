#!/usr/bin/env python3
"""Run ``evaluate_policy.py`` with random trajectories.

Creates a dataset of multiple random trajectories.  Then executes
``evaluate_policy.py`` on each of them and collects the log files in the
specified output directory.

To evaluate your policy, copy the ``evaluate_policy.py`` from this package and
replace the dummy policy there with your actual policy.

This is used for the evaluation of the submissions of the first phase of the
Real Robot Challenge 2021.  Make sure this script runs with your
``evaluate_policy.py`` script without any errors before submitting your code.
"""

# IMPORTANT:  DO NOT MODIFY THIS FILE!
# Submissions will be evaluate on our side with a similar script but not
# exactly this one.  To make sure that your code is compatible with our
# evaluation script, make sure it runs with this one without any modifications.

import argparse
import pathlib
import pickle
import subprocess
import sys
import typing

from trifinger_simulation.tasks import move_cube_on_trajectory as mct


def generate_test_set(num_trajectories: int) -> typing.List[str]:
    """Generate a test set of random trajectories.

    Args:
        num_trajectories:  Number of trajectories in the test set.

    Returns:
        List of random trajectories encoded as JSON strings.
    """
    return [
        mct.trajectory_to_json(mct.sample_goal())
        for i in range(num_trajectories)
    ]


def run_evaluate_policy(script_path: str, sample: str, action_log_file: str):
    """Run the evaluation script with the given sample.

    Args:
        script_path: Path to the evaluate_policy.py script.
        sample: Goal trajectory encoded as JSON string.
        action_log_file: Destination of the action log file.
    """
    cmd = [
        "python3",
        script_path,
        sample,
        action_log_file,
    ]
    subprocess.run(cmd, check=True)


def add_arguments(parser):
    parser.add_argument(
        "output_directory",
        type=pathlib.Path,
        help="Directory in which generated files are stored.",
    )
    parser.add_argument(
        "--exec",
        type=str,
        default="./evaluate_policy.py",
        help="Path to the evaluate_policy.py script.  Default: '%(default)s'",
    )


def main(output_directory: pathlib.Path, eval_executable: str):
    if not output_directory.is_dir():
        print(
            "'{}' does not exist or is not a directory.".format(
                output_directory
            )
        )
        sys.exit(1)

    # Number of goals that are sampled for the evaluation
    num_trajectories = 10

    logfile_tmpl = str(output_directory / "action_log_{:02d}.p")

    # generate n samples for each level
    test_data = generate_test_set(num_trajectories)

    # store samples
    sample_file = output_directory / "test_data.p"
    with open(sample_file, "wb") as fh:
        pickle.dump(test_data, fh, pickle.HIGHEST_PROTOCOL)

    # run "evaluate_move_cube_on_trajectory.py" for each sample
    for i, sample in enumerate(test_data):
        print("\n___Evaluate trajectory {}___".format(i))
        run_evaluate_policy(eval_executable, sample, logfile_tmpl.format(i))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    add_arguments(parser)
    args = parser.parse_args()

    main(args.output_directory, args.exec)
