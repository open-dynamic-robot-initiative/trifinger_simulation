#!/usr/bin/env python3
"""Simple Inverse kinematics demo."""
import numpy as np
import time
import math

import trifinger_simulation
from trifinger_simulation import sample, visual_objects
import ipdb
import traceback
import sys

def main():
    time_step = 0.004
    finger = trifinger_simulation.SimFinger(
        finger_type="trifingerpro",
        time_step=time_step,
        enable_visualization=True,
    )

    init_pos = np.array([0, 0.9, -1.7] * finger.number_of_fingers)
    finger.reset_finger_positions_and_velocities(init_pos)

    tip_positions = finger.kinematics.forward_kinematics(init_pos)

    # move the tips back and forth on the y-axis
    joint_pos = init_pos
    sign = +1
    while True:
        n_steps = int(1 / time_step)
        sign *= -1
        for _ in range(n_steps):
            finger_action = finger.Action(position=joint_pos)
            t = finger.append_desired_action(finger_action)
            obs = finger.get_observation(t)
            time.sleep(time_step)

            for i in range(len(tip_positions)):
                tip_positions[i][1] += sign * 0.05 / n_steps
            joint_pos, err = finger.kinematics.inverse_kinematics(
                tip_positions, obs.position
            )
            print("error:", [round(np.linalg.norm(e), 4) for e in err])

def random_ik():
    finger = trifinger_simulation.SimFinger(
        finger_type="fingerone",
        enable_visualization=True,
    )
    control_rate_s = 0.02
    goal = visual_objects.Marker(
            number_of_goals=1
        )
    observation = finger.reset_finger_positions_and_velocities(
        [0.0, 0.0, 0.0]
    )
    desired_finger_joint_positions = observation.position
    time.sleep(5)

    while True:
        tip_pos = sample.random_position_in_arena(
            height_limits=0.0325,
            angle_limits=(-2 * math.pi, 2 * math.pi),
            radius_limits=(0.0, 0.15),
        )
        goal.set_state([tip_pos])
        desired_finger_joint_positions, err = \
            finger.kinematics.inverse_kinematics_one_finger(
                finger_idx=0,
                tip_target_position=tip_pos,
                joint_angles_guess=observation.position,
            )
        for _ in range(int(
            round(control_rate_s / finger.time_step_s))):
            t = finger.append_desired_action(
                finger.Action(position=desired_finger_joint_positions)
            )
            observation = finger.get_observation(t)
            print(
                observation.position - desired_finger_joint_positions)
            time.sleep(finger.time_step_s)


if __name__ == "__main__":
    # main()
    try:
        random_ik()
        # main()
    except:
        extype, value, tb = sys.exc_info()
        traceback.print_exc()
        ipdb.post_mortem(tb)

