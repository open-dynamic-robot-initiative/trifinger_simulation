#!/usr/bin/env python3
"""Replay actions for a given logfile and verify final object pose.

The log file is a JSON file as produced by
`trifinger_simulation.TriFingerPlatform.store_action_log()` which contains the
initial state, a list of all applied actions and the final state of the object.

The simulation is initialised according to the given initial pose and the
actions are applied one by one.  In the end, it is verified if the final object
pose in the simulation matches the one in the log file.

The accumulated reward is computed based on the given goal pose and printed in
the end.

Both initial and goal pose are given as JSON strings with keys "position" and
"orientation" (as quaternion).  Example:

    {"position": [-0.03, 0.07, 0.05], "orientation": [0.0, 0.0, 0.68, -0.73]}

"""
import argparse
import pickle
import sys
import numpy as np

from trifinger_simulation import trifinger_platform
from trifinger_simulation.tasks import move_cube


def replay_action_log(logfile, difficulty, initial_pose, goal_pose):

    with open(logfile, "rb") as fh:
        log = pickle.load(fh)

    initial_object_pose = move_cube.Pose.from_json(initial_pose)
    goal_pose = move_cube.Pose.from_json(goal_pose)

    # verify that the initial object pose matches with the one in the log file
    try:
        np.testing.assert_array_almost_equal(
            initial_object_pose.position,
            log["initial_object_pose"].position,
            err_msg=(
                "Given initial object position does not match with log file."
            ),
        )
        np.testing.assert_array_almost_equal(
            initial_object_pose.orientation,
            log["initial_object_pose"].orientation,
            err_msg=(
                "Given initial object orientation does not match with log"
                " file."
            ),
        )
    except AssertionError as e:
        print("Failed.", file=sys.stderr)
        print(e, file=sys.stderr)
        sys.exit(1)

    platform = trifinger_platform.TriFingerPlatform(
        visualization=False, initial_object_pose=initial_object_pose
    )

    # verify that the robot is initialized to the same position as in the log
    # file
    initial_robot_position = platform.get_robot_observation(0).position
    try:
        np.testing.assert_array_almost_equal(
            initial_robot_position,
            log["initial_robot_position"],
            err_msg=("Initial robot position does not match with log file."),
        )
    except AssertionError as e:
        print("Failed.", file=sys.stderr)
        print(e, file=sys.stderr)
        sys.exit(1)

    # verify that the number of logged actions matches with the episode length
    n_actions = len(log["actions"])
    assert (
        n_actions == move_cube.episode_length
    ), "Number of actions in log does not match with expected episode length."

    accumulated_reward = 0
    for logged_action in log["actions"]:
        action = logged_action["action"]

        t = platform.append_desired_action(action)

        robot_obs = platform.get_robot_observation(t)
        cube_pose = platform.get_camera_observation(t).filtered_object_pose
        reward = -move_cube.evaluate_state(goal_pose, cube_pose, difficulty)
        accumulated_reward += reward

        assert logged_action["t"] == t

        np.testing.assert_array_almost_equal(
            robot_obs.position,
            logged_action["robot_observation"].position,
            err_msg=(
                "Step %d: Recorded robot position does not match with"
                " the one achieved by the replay" % t
            ),
        )
        np.testing.assert_array_almost_equal(
            robot_obs.torque,
            logged_action["robot_observation"].torque,
            err_msg=(
                "Step %d: Recorded robot torque does not match with"
                " the one achieved by the replay" % t
            ),
        )
        np.testing.assert_array_almost_equal(
            robot_obs.velocity,
            logged_action["robot_observation"].velocity,
            err_msg=(
                "Step %d: Recorded robot velocity does not match with"
                " the one achieved by the replay" % t
            ),
        )

        np.testing.assert_array_almost_equal(
            cube_pose.position,
            logged_action["object_pose"].position,
            err_msg=(
                "Step %d: Recorded object position does not match with"
                " the one achieved by the replay" % t
            ),
        )
        np.testing.assert_array_almost_equal(
            cube_pose.orientation,
            logged_action["object_pose"].orientation,
            err_msg=(
                "Step %d: Recorded object orientation does not match with"
                " the one achieved by the replay" % t
            ),
        )

    cube_pose = platform.get_camera_observation(t).object_pose
    final_pose = log["final_object_pose"]["pose"]

    print("Accumulated Reward:", accumulated_reward)

    # verify that actual and logged final object pose match
    try:
        np.testing.assert_array_almost_equal(
            cube_pose.position,
            final_pose.position,
            decimal=3,
            err_msg=(
                "Recorded object position does not match with the one"
                " achieved by the replay"
            ),
        )
        np.testing.assert_array_almost_equal(
            cube_pose.orientation,
            final_pose.orientation,
            decimal=3,
            err_msg=(
                "Recorded object orientation does not match with the one"
                " achieved by the replay"
            ),
        )
    except AssertionError as e:
        print("Failed.", file=sys.stderr)
        print(e, file=sys.stderr)
        sys.exit(1)

    print("Passed.")

    return accumulated_reward


def add_arguments(parser):
    parser.add_argument(
        "--logfile",
        "-l",
        required=True,
        type=str,
        help="Path to the log file.",
    )
    parser.add_argument(
        "--difficulty",
        "-d",
        required=True,
        type=int,
        help="The difficulty level of the goal (for reward computation).",
    )
    parser.add_argument(
        "--initial-pose",
        "-i",
        required=True,
        type=str,
        metavar="JSON",
        help="Initial pose of the cube as JSON string.",
    )
    parser.add_argument(
        "--goal-pose",
        "-g",
        required=True,
        type=str,
        metavar="JSON",
        help="Goal pose of the cube as JSON string.",
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    add_arguments(parser)
    args = parser.parse_args()

    replay_action_log(
        args.logfile, args.difficulty, args.initial_pose, args.goal_pose
    )
