#!/usr/bin/env python3
"""
To demonstrate reaching a randomly set target point in the arena using torque
control by directly specifying the position of the target only.
"""
import time
import math
import numpy as np

from pybullet_fingers import sim_finger


def main():
    time_step = 0.004

    finger_type = "tri"
    num_fingers = 1 if finger_type == "single" else 3

    action_bounds = {
        "low": np.array([-math.radians(90),
                         -math.radians(90),
                         -math.radians(172)] * num_fingers),
        "high": np.array([math.radians(90),
                          math.radians(100),
                          math.radians(-2)] * num_fingers),
    }
    finger = sim_finger.Finger(time_step, True, finger_type, action_bounds)
    finger.reset_finger()

    finger.display_goal()
    while True:
        desired_joint_positions = \
            finger.sample_random_joint_positions_for_reaching()

        # for visualization
        desired_tip_positions = finger.forward_kinematics(
            np.array(desired_joint_positions))
        finger.reset_goal_markers(desired_tip_positions)

        for _ in range(500):
            time.sleep(time_step)
            finger._apply_action(desired_joint_positions, "position")
            finger._step_simulation()

        end_effector_pos = np.asarray(finger.get_end_effector_position())
        err = end_effector_pos - np.concatenate(desired_tip_positions)
        print("Position error: %.4f   %s" % (np.linalg.norm(err), err))


if __name__ == "__main__":
    main()
