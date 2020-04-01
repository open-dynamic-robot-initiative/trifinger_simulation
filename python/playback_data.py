#!/usr/bin/env python3
"""Load position data from file and play back with pyBullet-visualization."""
import argparse
import numpy as np
import datetime
import time
import pickle
import copy

import transformations as tf
import pybullet

from pybullet_fingers.sim_finger import SimFinger


def sleep_until(until, accuracy=0.01):
    """
    Sleep until the given time.

    Args:
        until (datetime.datetime): Time until the function should sleep.
        accuracy (float): Accuracy with which it will check if the "until" time
            is reached.

    """
    while until > datetime.datetime.now():
        time.sleep(accuracy)


def main_single():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("data_file")
    args = parser.parse_args()

    finger = SimFinger(0.001, True, "single", None)
    finger.display_goal()

    def reset_finger(joint_position):
        for i, joint_id in enumerate(finger.revolute_joint_ids):
            pybullet.resetJointState(
                finger.finger_id, joint_id, joint_position[i]
            )

    with open(args.data_file, "rb") as file_handle:
        episodes = pickle.load(file_handle)

    start_time_data = None
    start_time_playback = datetime.datetime.now()

    # for episode_num, data in enumerate(episodes[1:]):
    for episode_num, data in enumerate(episodes[680:]):
        print("episode {}".format(episode_num))
        tip_goal = data["tip_goal"][0]
        trajectory = np.vstack(data["joint_positions"])
        timestamps = data["timestamps"]

        if not start_time_data:
            start_time_data = timestamps[0]

        timediff_data = timestamps[0] - start_time_data - 1
        sleep_until(
            start_time_playback + datetime.timedelta(seconds=timediff_data)
        )
        finger.reset_goal_markers([tip_goal])
        reset_finger(trajectory[0])

        for position, timestamp in zip(trajectory, timestamps):
            timediff_data = timestamp - start_time_data
            sleep_until(
                start_time_playback + datetime.timedelta(seconds=timediff_data)
            )

            reset_finger(position)


def main_three():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("data_file_0")
    parser.add_argument("data_file_120")
    parser.add_argument("data_file_240")
    args = parser.parse_args()

    rot_z_120 = tf.euler_matrix(0, 0, np.deg2rad(-120))[:3, :3]
    y_shift = np.array([0, 0.04, 0])

    finger = SimFinger(0.004, True, "tri", None)
    finger.display_goal()

    def reset_finger(joint_position):
        for i, joint_id in enumerate(finger.revolute_joint_ids):
            pybullet.resetJointState(
                finger.finger_id, joint_id, joint_position[i]
            )

    data = []
    for filename in (args.data_file_0, args.data_file_120, args.data_file_240):
        with open(filename, "rb") as file_handle:
            data.append(pickle.load(file_handle))

    assert len(data[0]) == len(data[1]) == len(data[2])

    episodes = []
    for i in range(1, len(data[0])):
        goals = [
            (data[0][i]["tip_goal"][0] + y_shift),
            rot_z_120 @ (data[1][i]["tip_goal"][0] + y_shift),
            rot_z_120 @ rot_z_120 @ (data[2][i]["tip_goal"][0] + y_shift),
        ]

        initial_position = np.concatenate(
            [
                data[0][i]["joint_positions"][0],
                data[1][i]["joint_positions"][0],
                data[2][i]["joint_positions"][0],
            ]
        )

        joint_positions = []
        for f in range(3):
            for stamp, pos in zip(
                data[f][i]["timestamps"], data[f][i]["joint_positions"]
            ):
                joint_positions.append((f, stamp, pos))
        joint_positions = sorted(joint_positions, key=lambda x: x[1])

        episodes.append(
            {
                "tip_goals": goals,
                "joint_positions": joint_positions,
                "initial_position": initial_position,
            }
        )

    input()

    start_time_data = None
    start_time_playback = datetime.datetime.now()

    for episode_num, data in enumerate(episodes):
        print("episode {}".format(episode_num))
        tip_goals = data["tip_goals"]
        trajectory = data["joint_positions"]

        episode_start_time = trajectory[0][1]

        if not start_time_data:
            start_time_data = episode_start_time

        timediff_data = episode_start_time - start_time_data - 1
        sleep_until(
            start_time_playback + datetime.timedelta(seconds=timediff_data)
        )
        finger.reset_goal_markers(tip_goals)
        reset_finger(data["initial_position"])

        current_pos = copy.copy(data["initial_position"])

        for finger_idx, timestamp, position in trajectory:
            timediff_data = timestamp - start_time_data
            sleep_until(
                start_time_playback + datetime.timedelta(seconds=timediff_data)
            )

            slice_start = finger_idx * 3
            current_pos[slice_start : slice_start + 3] = position

            reset_finger(current_pos)


if __name__ == "__main__":
    main_three()
