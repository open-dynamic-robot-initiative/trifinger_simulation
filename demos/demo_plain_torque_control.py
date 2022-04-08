#!/usr/bin/env python3
"""Demonstrate how to run the simulated finger with torque control."""
import time
import numpy as np

from trifinger_simulation import SimFinger


if __name__ == "__main__":
    finger = SimFinger(
        finger_type="fingerone",
        enable_visualization=True,
    )

    # Send a constant torque to the joints, switching direction periodically.
    torque = np.array([0.0, 0.3, 0.3])
    while True:
        time.sleep(finger.time_step_s)

        action = finger.Action(torque=torque)
        t = finger.append_desired_action(action)
        observation = finger.get_observation(t)

        # invert the direction of the command every 100 steps
        if t % 100 == 0:
            torque *= -1
