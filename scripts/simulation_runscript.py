#!/usr/bin/env python3
"""
Execute `evaluate_policy.py` in the current working directory.

An executable called `evaluate_policy.py` needs to be present in the current
working directory (use `--run` to change the name).

The following arguments will be passed to the executable:

1) The difficulty level (see `--difficulty`).
2) The initial object pose.
3) The goal pose for the object.

The poses are given as JSON strings with two keys "position" and "orientation"
which contain the (x, y, z) goal position and a quaternion (x, y, z, w) with
the goal orientation w.r.t. the world coordinate frame.  Example:

    {"position": [-0.03, 0.07, 0.05], "orientation": [0.0, 0.0, 0.68, -0.73]}

If no goal/initial pose is passed to this script, the pose is sampled based on
the given difficulty level.
"""
import argparse
import json
import os
import subprocess
import sys

from pybullet_fingers.tasks import move_cube


def validate_json_pose(json_str):
    """Validate if the given string is a valid pose in JSON format.

    Args:
        json_str (str):  The string to check.

    Raises:
        ValueError:  If the given string is not valid.
    """

    try:
        pose = json.loads(json_str)
    except Exception as e:
        raise ValueError("Not a valid JSON string. %s" % e)

    for key in ("position", "orientation"):
        if key not in pose:
            raise ValueError("Does not contain key '%s'." % key)
    if type(pose["position"]) != list or len(pose["position"]) != 3:
        raise ValueError("'position' needs to be a list of three values [x, y, z].")
    if type(pose["orientation"]) != list or len(pose["orientation"]) != 4:
        raise ValueError("'orientation' needs to be a quaternion [x, y, z, w]")

    move_cube.validate_goal(move_cube.Pose.from_dict(pose))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--difficulty",
        "-d",
        type=int,
        choices=[1, 2, 3, 4],
        required=True,
        help="""
        Difficulty level of the goal.  Used for sampling goals.  However, it
        also needs to be specified when passing a fixed goal (see `--goal`) as
        it is needed for the reward computation.
        """,
    )
    parser.add_argument(
        "--goal",
        "-g",
        type=str,
        metavar="JSON_STRING",
        help="""
        Pass a custom goal as a JSON string using the format described above.
        If not set, a random goal is sampled based on the specified difficulty
        level.
        """,
    )
    parser.add_argument(
        "--initial-pose",
        "-i",
        type=str,
        metavar="JSON_STRING",
        help="""
        Initial pose of the object as a JSON string using the format described
        above.  If not specified, a random pose is sampled.
        """,
    )
    parser.add_argument(
        "--run",
        type=str,
        metavar="EXECUTABLE",
        default="./evaluate_policy.py",
        help="""
        Command that is executed.  Defaults to "%(default)s".
        """,
    )
    args = parser.parse_args()

    run_command = args.run

    if not os.path.exists(run_command):
        print(
            "ERROR: No executable '%s' found."
            " It needs to be in the current working directory" % run_command
        )
        sys.exit(1)

    if args.goal is not None:
        # validate string
        try:
            validate_json_pose(args.goal)
        except Exception as e:
            print("ERROR: Invalid goal. ", e)
            sys.exit(1)

        goal_string = args.goal
    else:
        goal_pose = move_cube.sample_goal(args.difficulty)
        goal_string = goal_pose.to_json()

    if args.initial_pose is not None:
        # validate string
        try:
            validate_json_pose(args.initial_pose)
        except Exception as e:
            print("ERROR: Invalid initial pose. ", e)
            sys.exit(1)

        initial_pose_string = args.initial_pose
    else:
        initial_pose = move_cube.sample_goal(-1)
        initial_pose_string = initial_pose.to_json()

    subprocess.run(
        [run_command, str(args.difficulty), initial_pose_string, goal_string]
    )
