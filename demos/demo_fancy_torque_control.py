#!/usr/bin/env python3

"""
To demonstrate reaching a randomly set target point in the arena using torque
control by directly specifying the position of the target only.
"""
import time

from pybullet_fingers import sim_finger


def main():
    time_step = 0.004

    finger = sim_finger.Finger(time_step, True, "tri", "triangle")

    finger.display_goal()
    while True:
        desired_position = finger.sample_random_position_in_arena()
        tip_positions = finger.get_tip_positions_around_position(
            desired_position)
        finger.reset_goal_markers(tip_positions)

        joint_positions = finger.pybullet_inverse_kinematics(tip_positions)

        time.sleep(time_step)
        for _ in range(500):
            finger._apply_action(joint_positions, "position")
            finger._step_simulation()
            time.sleep(time_step)
    finger.reset_finger()


if __name__ == "__main__":
    main()
