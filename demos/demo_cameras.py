#!/usr/bin/env python3
"""Demo showing how add cameras in the TriFinger simulation."""
import numpy as np
import cv2

from pybullet_fingers import sim_finger, sample, camera


def main():
    time_step = 0.004
    finger = sim_finger.SimFinger(
        finger_type="trifingerone",
        time_step=time_step,
        enable_visualization=True,
        )

    # Important: The cameras need the be created _after_ the simulation is
    # initialized.
    cameras = camera.TriFingerCameras()

    # Move the fingers to random positions
    while True:
        goal = np.array(
            sample.random_joint_positions(
                number_of_fingers=3,
                lower_bounds=[-1, -1, -2],
                upper_bounds=[1, 1, 2],
            )
        )
        finger_action = finger.Action(position=goal)

        for _ in range(50):
            t = finger.append_desired_action(finger_action)
            finger.get_observation(t)

            images = cameras.get_images()
            # images are rgb --> convert to bgr for opencv
            images = [cv2.cvtColor(img, cv2.COLOR_RGB2BGR) for img in images]
            cv2.imshow("camera60", images[0])
            cv2.imshow("camera180", images[1])
            cv2.imshow("camera300", images[2])
            cv2.waitKey(int(time_step * 1000))


if __name__ == "__main__":
    main()
