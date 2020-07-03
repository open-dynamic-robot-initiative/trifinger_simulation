#!/usr/bin/env python3
"""Simple demo on how to use the TriFingerPlatform interface."""
import time

import cv2
import numpy as np

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

        # apply action for a few steps, so the fingers can move to the target
        # position and stay there for a while
        for _ in range(250):
            t = platform.append_desired_action(finger_action)
            time.sleep(0.001)

        # show the latest observations
        robot_observation = platform.get_robot_observation(t)
        print("Finger0 Position: %s" % robot_observation.position[:3])

        cube_pose = platform.get_object_pose(t)
        print("Cube Position: %s" % cube_pose.position)

        camera_observation = platform.get_camera_observation(t)
        for i, name in enumerate(("camera60", "camera180", "camera300")):
            # simulation provides images in RGB but OpenCV expects BGR
            img = cv2.cvtColor(
                camera_observation.cameras[i].image, cv2.COLOR_RGB2BGR
            )
            cv2.imshow(name, img)
        cv2.waitKey(1)

        print()


if __name__ == "__main__":
    main()
