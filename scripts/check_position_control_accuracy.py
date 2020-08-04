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
    parser.add_argument("--plot", action="store_true")
    args = parser.parse_args()

    time_step = args.time_step

    finger = sim_finger.SimFinger(
        time_step=time_step,
        enable_visualization=True,
        finger_type="fingerone",
    )
    # set the finger to a reasonable start position
    finger.reset_finger_positions_and_velocities([0, -0.7, -1.5])

    errors = []
    for _ in range(100):
        target_position = np.array(
            sample.random_joint_positions(number_of_fingers=1)
        )

        action = finger.Action(position=target_position)
        positions = []
        for _ in range(500):
            t = finger.append_desired_action(action)
            observation = finger.get_observation(t)
            positions.append(observation.position)

        observation = finger.get_observation(t)
        error = np.abs(observation.position - target_position)
        errors.append(error)
        print("Position error: {}".format(error))

        if args.plot:
            positions = np.vstack(positions)
            plt.plot(positions[:, 1], "b")
            plt.hlines(target_position[1], 0, 500)
            plt.show()

    print("==============================================================")
    print("Mean error: {}".format(np.mean(errors, axis=0)))
    print("\n\n")
