#!/usr/bin/env python3
"""Simple demo on how to use the TriFingerPlatform interface."""
import numpy as np
import time

from pybullet_fingers import trifinger_platform, sample


def main():
    platform = trifinger_platform.TriFingerPlatform(visualization=True)

    # Move the fingers to random positions so that the cube is kicked around
    # (and thus it's position changes).
    while True:
        goal = np.array(
            sample.random_joint_positions(
                number_of_fingers=3,
                lower_bounds=[-1, -1, -2],
                upper_bounds=[1, 1, 2],
            )
        )
        finger_action = platform.Action(position=goal)

        for _ in range(250):
            t = platform.append_desired_action(finger_action)
            time.sleep(0.001)

        # print the latest observations
        robot_observation = platform.get_robot_observation(t)
        print("Finger0 Position: %s" % robot_observation.position[:3])

        cube_pose = platform.get_object_pose(t)
        print("Cube Position: %s" % cube_pose.position)

        print()


if __name__ == "__main__":
    main()
