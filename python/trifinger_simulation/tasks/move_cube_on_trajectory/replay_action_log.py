#!/usr/bin/env python3
"""Replay actions for a given logfile, verify steps and compute reward.

The log file is a pickle file as produced by
:meth:`trifinger_simulation.TriFingerPlatform.store_action_log` which contains
a list of applied actions and observations of all steps.

The simulation is initialised to the same state as during evaluation.  Then the
actions are applied one by one, verifying the resulting robot and object state
after each step.  If the simulation was only accessed via the provided
interface during evaluation, the replay should result in exactly the same
states.

For each step the reward is computed.  The cumulative reward over all steps is
printed in the end.
"""
import argparse
import json
import pickle
import sys
import numpy as np

from trifinger_simulation import trifinger_platform
from trifinger_simulation.tasks import move_cube_on_trajectory as mct


def replay_action_log(logfile: str, trajectory: mct.Trajectory) -> float:
    with open(logfile, "rb") as fh:
        log = pickle.load(fh)

    # initialize cube at the centre
    initial_object_pose = mct.move_cube.Pose(
        position=mct.INITIAL_CUBE_POSITION
    )

    platform = trifinger_platform.TriFingerPlatform(
        visualization=False, initial_object_pose=initial_object_pose
    )

    # verify that the robot is initialized to the same position as in the log
    # file
    initial_robot_position = platform.get_robot_observation(0).position
    try:
        np.testing.assert_array_equal(
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
        n_actions == mct.EPISODE_LENGTH
    ), "Number of actions in log does not match with expected episode length."

    accumulated_reward = 0
    for logged_action in log["actions"]:
        action = logged_action["action"]

        t = platform.append_desired_action(action)

        robot_obs = platform.get_robot_observation(t)
        camera_obs = platform.get_camera_observation(t)
        # ensure we got a camera observation with object pose (mostly for mypy)
        assert isinstance(
            camera_obs, trifinger_platform.TriCameraObjectObservation
        )

        cube_pose = camera_obs.filtered_object_pose
        reward = -mct.evaluate_state(trajectory, t, cube_pose.position)
        accumulated_reward += reward

        assert logged_action["t"] == t

        np.testing.assert_array_equal(
            robot_obs.position,
            logged_action["robot_observation"].position,
            err_msg=(
                "Step %d: Recorded robot position does not match with"
                " the one achieved by the replay" % t
            ),
        )
        np.testing.assert_array_equal(
            robot_obs.torque,
            logged_action["robot_observation"].torque,
            err_msg=(
                "Step %d: Recorded robot torque does not match with"
                " the one achieved by the replay" % t
            ),
        )
        np.testing.assert_array_equal(
            robot_obs.velocity,
            logged_action["robot_observation"].velocity,
            err_msg=(
                "Step %d: Recorded robot velocity does not match with"
                " the one achieved by the replay" % t
            ),
        )

        np.testing.assert_array_equal(
            cube_pose.position,
            logged_action["object_pose"].position,
            err_msg=(
                "Step %d: Recorded object position does not match with"
                " the one achieved by the replay" % t
            ),
        )
        np.testing.assert_array_equal(
            cube_pose.orientation,
            logged_action["object_pose"].orientation,
            err_msg=(
                "Step %d: Recorded object orientation does not match with"
                " the one achieved by the replay" % t
            ),
        )

    camera_obs = platform.get_camera_observation(t)
    assert isinstance(
        camera_obs, trifinger_platform.TriCameraObjectObservation
    )
    cube_pose = camera_obs.object_pose
    final_pose = log["final_object_pose"]["pose"]

    print("Accumulated Reward:", accumulated_reward)

    # verify that actual and logged final object pose match
    try:
        np.testing.assert_array_equal(
            cube_pose.position,
            final_pose.position,
            err_msg=(
                "Recorded object position does not match with the one"
                " achieved by the replay"
            ),
        )
        np.testing.assert_array_equal(
            cube_pose.orientation,
            final_pose.orientation,
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
        "--trajectory",
        "-t",
        required=True,
        type=json.loads,
        metavar="JSON",
        help="Goal trajectory of the cube as JSON string.",
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    add_arguments(parser)
    args = parser.parse_args()

    replay_action_log(args.logfile, args.trajectory)
