#!/usr/bin/env python3
"""Simple Inverse kinematics demo."""
import numpy as np
import time

import trifinger_simulation


def main():
    time_step = 0.001
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


if __name__ == "__main__":
    main()
