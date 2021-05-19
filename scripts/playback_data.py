#!/usr/bin/env python3
"""Load position data from file and play back with pyBullet-visualization."""
import sys
import argparse
import numpy as np
import datetime
import pickle
import copy

import transformations as tf
import pybullet

from trifinger_simulation.sim_finger import SimFinger
from trifinger_simulation import visual_objects
from trifinger_simulation.gym_wrapper import utils
from trifinger_simulation import finger_types_data


def main_finger(finger_type, data_file):
    finger = SimFinger(enable_visualization=True, finger_type=finger_type)
    goal_marker = visual_objects.Marker(number_of_goals=1)

    def reset_finger(joint_position):
        for i, joint_id in enumerate(finger.pybullet_joint_indices):
            pybullet.resetJointState(
                finger.finger_id, joint_id, joint_position[i]
            )

    with open(data_file, "rb") as file_handle:
        episodes = pickle.load(file_handle)

    input("Press enter to start playback")

    start_time_data = None
    start_time_playback = datetime.datetime.now()

    for episode_num, data in enumerate(episodes[1:]):
        print("episode {}".format(episode_num))

        tip_goal = data["tip_goal"][0]
        trajectory = np.vstack(data["joint_positions"])
        timestamps = data["timestamps"]

        if not start_time_data:
            start_time_data = timestamps[0]

        timediff_data = timestamps[0] - start_time_data - 1
        utils.sleep_until(
            start_time_playback + datetime.timedelta(seconds=timediff_data)
        )
        goal_marker.set_state(tip_goal)
        reset_finger(trajectory[0])

        for position, timestamp in zip(trajectory, timestamps):
            timediff_data = timestamp - start_time_data
            utils.sleep_until(
                start_time_playback + datetime.timedelta(seconds=timediff_data)
            )

            reset_finger(position)


def trifinger_goal_space_transforms(finger_type, data):

    rot_z_120 = tf.euler_matrix(0, 0, np.deg2rad(-120))[:3, :3]

    if finger_type == "trifingerone":
        y_shift = np.array([0, 0.04, 0])
        for i in range(1, len(data[0])):
            goals = [
                (data[0][i]["tip_goal"][0] + y_shift),
                rot_z_120 @ (data[1][i]["tip_goal"][0] + y_shift),
                rot_z_120 @ rot_z_120 @ (data[2][i]["tip_goal"][0] + y_shift),
            ]

    elif finger_type == "trifingeredu":
        rot_z_90 = tf.euler_matrix(0, 0, np.deg2rad(-90))[:3, :3]
        x_shift = np.array([0.0455, 0, 0])
        for i in range(1, len(data[0])):
            goals = [
                (rot_z_90 @ data[0][i]["tip_goal"][0] - x_shift),
                rot_z_120 @ rot_z_90 @ (data[1][i]["tip_goal"][0] - x_shift),
                rot_z_120
                @ rot_z_120
                @ rot_z_90
                @ (data[2][i]["tip_goal"][0] - x_shift),
            ]
    return goals


def main_trifinger(finger_type, data_file_0, data_file_120, data_file_240):

    finger = SimFinger(enable_visualization=True, finger_type=finger_type)
    goal_marker = visual_objects.Marker(number_of_goals=3)

    def reset_finger(joint_position):
        for i, joint_id in enumerate(finger.pybullet_joint_indices):
            pybullet.resetJointState(
                finger.finger_id, joint_id, joint_position[i]
            )

    data = []
    for filename in (data_file_0, data_file_120, data_file_240):
        with open(filename, "rb") as file_handle:
            data.append(pickle.load(file_handle))

    assert len(data[0]) == len(data[1]) == len(data[2])

    episodes = []
    for i in range(1, len(data[0])):
        goals = trifinger_goal_space_transforms(finger_type, data)

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

    input("Press enter to start playback")

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
        utils.sleep_until(
            start_time_playback + datetime.timedelta(seconds=timediff_data)
        )
        goal_marker.set_state(tip_goals)
        reset_finger(data["initial_position"])

        current_pos = copy.copy(data["initial_position"])

        for finger_idx, timestamp, position in trajectory:
            timediff_data = timestamp - start_time_data
            utils.sleep_until(
                start_time_playback + datetime.timedelta(seconds=timediff_data)
            )

            slice_start = finger_idx * 3
            current_pos[slice_start : slice_start + 3] = position

            reset_finger(current_pos)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("finger_type")
    args = parser.parse_args(sys.argv[1:2])

    finger_type = finger_types_data.check_finger_type(args.finger_type)
    num_fingers = finger_types_data.get_number_of_fingers(finger_type)

    parser = argparse.ArgumentParser(description=__doc__)
    if num_fingers == 1:
        parser.add_argument("data_file")
    elif num_fingers == 3:
        parser.add_argument("data_file_0")
        parser.add_argument("data_file_120")
        parser.add_argument("data_file_240")
    args = parser.parse_args(sys.argv[2:])

    if num_fingers == 3:
        main_trifinger(
            finger_type,
            args.data_file_0,
            args.data_file_120,
            args.data_file_240,
        )
    else:
        main_finger(finger_type, args.data_file)


if __name__ == "__main__":
    main()
