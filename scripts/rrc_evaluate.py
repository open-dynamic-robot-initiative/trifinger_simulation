#!/usr/bin/env python3
"""Run RRC Simulation Phase Evaluation in a Singularity container.

Copies the specified package into the workspace created in the output
directory, builds it using ``colcon build`` and then runs policy evaluation and
replay assuming that a proper ``evaluate_policy.py`` exists in the root of the
given package.

Build and evaluation is executed inside the specified Singularity image which
is run with "--containall --cleanenv" to minimise any influence of the host
system.  The only directories that are mounted are the package directory
(read-only) and the output directory.
"""
import argparse
import pathlib
import subprocess
import sys


#: The task that is evaluated
TASK = "move_cube_on_trajectory"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--singularity-image",
        "-s",
        type=pathlib.Path,
        help="Path to the Singularity image.",
    )
    parser.add_argument(
        "--package",
        "-p",
        type=pathlib.Path,
        help="""Path to the package containing the policy.  This package is
            expected to have a file `evaluate_policy.py` in its root directory.
        """,
    )
    parser.add_argument(
        "--output-dir",
        "-o",
        type=pathlib.Path,
        help="""Output directory in which log files are saved.  CAUTION:
            existing files in this directory may be deleted!
        """,
    )
    args = parser.parse_args()

    # some basic checks of the input
    if not args.singularity_image.exists():
        print(
            "The singularity image {} does not exist.".format(
                args.singularity_image
            ),
            file=sys.stderr,
        )
        return 1
    if not args.package.is_dir():
        print("{} is not a directory.".format(args.package), file=sys.stderr)
        return 1
    if not args.output_dir.is_dir():
        print("{} is not a directory".format(args.output_dir), file=sys.stderr)
        return 1

    eval_script = (
        "rm -rf /ws/{install,build,log,src} && "  # wipe existing workspace
        "mkdir -p /ws/src && "
        "mkdir -p /ws/output && "
        "cp -r /input /ws/src/pkg && "
        "cd /ws && "
        "colcon build && "
        ". install/local_setup.bash && "
        f"python3 -m trifinger_simulation.tasks.{TASK}"
        " evaluate_and_check --exec /ws/src/pkg/evaluate_policy.py /ws/output"
    )

    singularity_cmd = [
        "singularity",
        "run",
        "--containall",
        "--cleanenv",
        "-B",
        "{}:/input:ro,{}:/ws".format(args.package, args.output_dir),
        args.singularity_image,
        eval_script,
    ]

    subprocess.run(singularity_cmd, check=True)

    return 0


if __name__ == "__main__":
    sys.exit(main())
