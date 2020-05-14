#!/usr/bin/env python3

"""
To demonstrate reaching a randomly set target point in the arena using torque
control by directly specifying the position of the target only.
"""
import argparse
import numpy as np
import random
import time
from pybullet_fingers import sim_finger, visual_objects, sample


def main():

    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument(
        "--control_mode",
        default="position",
        choices=["position", "torque"],
        help="Specify position or torque as the control mode.",
    )
    argparser.add_argument(
        "--finger_type",
        default="tri",
        choices=["single", "tri"],
        help="Specify type of finger as single or tri.",
    )
    args = argparser.parse_args()
    time_step = 0.004

    finger = sim_finger.SimFinger(time_step, True, args.finger_type)
    if args.finger_type == "single":
        num_fingers = 1
    elif args.finger_type == "tri":
        num_fingers = 3
    if args.control_mode == "position":
        position_goals = visual_objects.Marker(number_of_goals=num_fingers)

    while True:

        if args.control_mode == "position":
            desired_joint_positions = np.array(
                sample.random_joint_positions(number_of_fingers=num_fingers)
            )
            finger_action = finger.Action(position=desired_joint_positions)
            # visualize the goal position of the finger tip
            position_goals.set_state(
                finger.pinocchio_utils.forward_kinematics(
                    desired_joint_positions
                )
            )
        if args.control_mode == "torque":
            desired_joint_torques = [random.random()] * 3 * num_fingers
            finger_action = finger.Action(torque=desired_joint_torques)

        for _ in range(20):
            t = finger.append_desired_action(finger_action)
            finger.get_observation(t)
            time.sleep(time_step)


if __name__ == "__main__":
    main()
