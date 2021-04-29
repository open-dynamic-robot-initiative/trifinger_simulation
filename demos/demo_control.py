#!/usr/bin/env python3

"""
To demonstrate reaching a randomly set target point in the arena using torque
control by directly specifying the position of the target only.
"""
import argparse
import numpy as np
import random
import time
from trifinger_simulation import (
    sim_finger,
    visual_objects,
    sample,
    finger_types_data,
)


def main():
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument(
        "--control-mode",
        default="position",
        choices=["position", "torque"],
        help="Specify position or torque as the control mode.",
    )
    argparser.add_argument(
        "--finger-type",
        default="trifingerone",
        choices=finger_types_data.get_valid_finger_types(),
        help="Specify a valid finger type",
    )
    args = argparser.parse_args()
    time_step = 0.001

    finger = sim_finger.SimFinger(
        finger_type=args.finger_type,
        time_step=time_step,
        enable_visualization=True,
    )
    num_fingers = finger.number_of_fingers

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
                finger.kinematics.forward_kinematics(desired_joint_positions)
            )
        if args.control_mode == "torque":
            desired_joint_torques = [random.random()] * 3 * num_fingers
            finger_action = finger.Action(torque=desired_joint_torques)

        # pursue this goal for one second
        for _ in range(int(1 / time_step)):
            t = finger.append_desired_action(finger_action)
            finger.get_observation(t)
            time.sleep(time_step)


if __name__ == "__main__":
    main()
