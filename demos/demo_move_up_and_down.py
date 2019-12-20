#!/usr/bin/env python3

"""
To set more uniformly varying target positions for the finger,
use this.

..note::
  This is based on position control. Can be used as a first test, as
  position control to random positions requires some adjustment of the
  number of action_repetitions.
"""
import sys
import math
import time

import pybullet

from pybullet_fingers import sim_finger


def run(finger, action):
    for _ in range(200):
        finger._apply_action(action, "position")
        finger._step_simulation()
        time.sleep(0.001)

def compute_tip_target_position(angle, t):
    radial_distance = 0.034
    finger_height = 0.34
    return [radial_distance * math.sin(math.radians(angle)),
            radial_distance * math.cos(math.radians(angle)),
            finger_height/2 * math.sin(t) + finger_height/2]


if __name__ == "__main__":

    if len(sys.argv) > 1:
        finger_type = sys.argv[1]
    else:
        finger_type = "trifinger"

    simulated_finger = sim_finger.Finger(finger_type=finger_type)

    if finger_type == "single":
        for t in range(100000):
            desired_position = compute_tip_target_position(0, t)

            run(simulated_finger, desired_position)

    if finger_type == "trifinger":
        for t in range(100000):
            desired_position = [compute_tip_target_position(0,t),
                                compute_tip_target_position(120,t),
                                compute_tip_target_position(240,t)]

            run(simulated_finger, desired_position)
