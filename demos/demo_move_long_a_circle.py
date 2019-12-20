#!/usr/bin/env python3

"""
To set more uniformly varying target positions for the finger,
use this.

..note::
  This is based on position control. Can be used as a first test, as
  position control to random positions requires some adjustment of the
  number of action_repetitions.
"""

import math
import time
import numpy as np

import pybullet

from pybullet_fingers import sim_finger

if __name__ == "__main__":
    simulated_finger = sim_finger.Finger()
    t = 0.
    while True:
        previous_desired_position = [
            0.15 * math.cos(t), 0.15 * math.sin(t), 0.08]
        t = t + 0.01
        current_desired_position = [
            0.15 * math.cos(t), 0.15 * math.sin(t), 0.08]
        simulated_finger.display_object_at_target(current_desired_position)
        pybullet.addUserDebugLine(
            previous_desired_position, current_desired_position, [
                0, 0, 0.3], 1, 7)

        for _ in range(50):
            simulated_finger._apply_action(current_desired_position, "torque")
            simulated_finger._step_simulation()
            time.sleep(0.001)
