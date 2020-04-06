#!/usr/bin/env python3
"""Demonstrate how to run the simulated finger with torque control."""
import time
import numpy as np

from pybullet_fingers import sim_finger


if __name__ == "__main__":

    time_step = 0.001

    finger = sim_finger.SimFinger(
        time_step=time_step,
        enable_visualization=True,
        finger_type="single",
        action_bounds=None,
    )
    # set the finger to a reasonable start position
    finger.reset_finger([0, -0.7, -1.5])

    torque = np.array([0.0, 0.3, 0.3])
    for t in range(10000000):
        time.sleep(time_step)

        # invert the direction of the command every 100 steps
        if t % 100 == 0:
            torque *= -1

        action = finger.Action(torque=torque)
        t = finger.append_desired_action(action)
        observation = finger.get_observation(t)
