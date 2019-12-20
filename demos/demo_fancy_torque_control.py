#!/usr/bin/env python3
"""
To demonstrate reaching a randomly set target point in the arena using torque
control by directly specifying the position of the target only.
"""
import math
import time
import numpy as np

from pybullet_fingers import sim_finger

if __name__ == "__main__":

    finger = sim_finger.Finger()

    for _ in range(3):
        desired_positon = finger.sample_random_position_in_arena()
        finger.display_object_at_target(desired_positon)

        time.sleep(0.001)
        for _ in range(1500):
            finger._apply_action(desired_positon, "torque")
            finger._step_simulation()
            time.sleep(0.001)
    finger.reset()
