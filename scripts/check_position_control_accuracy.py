#!/usr/bin/env python3
"""Move to random points with position control and print position error.

This can be used for testing different PD gains.
"""
import argparse
import numpy as np
import matplotlib.pyplot as plt

from trifinger_simulation import sim_finger, sample


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--time-step", "-t", type=float, required=True)
    parser.add_argument("--plot", type=int)
    args = parser.parse_args()

    time_step = args.time_step

    finger = sim_finger.SimFinger(
        time_step=time_step,
        enable_visualization=True,
        finger_type="fingerpro",
        # spawn robot higher up to avoid collisions with the table
        robot_position_offset=(0, 0, 0.5),
    )

    errors = []
    for _ in range(100):
        target_position = np.array(
            sample.random_joint_positions(
                number_of_fingers=1,
                lower_bounds=[-0.33, 0.0, -2.7],
                upper_bounds=[1.0, 1.57, 0.0],
            )
        )

        action = finger.Action(position=target_position)
        positions = []
        steps = 1000
        for _ in range(steps):
            t = finger.append_desired_action(action)
            observation = finger.get_observation(t)
            positions.append(observation.position)

        observation = finger.get_observation(t)
        error = np.abs(observation.position - target_position)
        errors.append(error)
        # print("Target position: {}".format(target_position))
        print("Position error:  {}".format(error))

        if args.plot is not None:
            target_line = target_position[args.plot]
            position_array = np.vstack(positions)
            plt.plot(position_array[:, args.plot], "b")
            plt.hlines(target_line, 0, steps)
            plt.axis((None, None, target_line - 0.1, target_line + 0.1))
            plt.show()

    print("==============================================================")
    print("Mean error: {}".format(np.mean(errors, axis=0)))
    print("\n\n")
