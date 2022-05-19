#!/usr/bin/env python3
"""Demo showing how to add cameras in the TriFinger simulation."""
import argparse
import pathlib

import numpy as np
import cv2

from trifinger_simulation import sim_finger, sample, camera


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--robot",
        "-r",
        choices=["trifingerone", "trifingerpro"],
        default="trifingerone",
        help="Which robot to use.  Default: %(default)s",
    )
    parser.add_argument(
        "--config-dir",
        "-c",
        type=pathlib.Path,
        help="""Path to the directory containing camera calibration files. This
            is optional, if not specified, some default values will be used.
        """,
    )
    parser.add_argument(
        "--param-file-format",
        type=str,
        default="camera{id}.yml",
        help="""Format of the camera parameter files. '{id}' is replaced with
            the camera id.  Default: %(default)s
        """,
    )
    args = parser.parse_args()

    time_step = 0.004
    finger = sim_finger.SimFinger(
        finger_type=args.robot,
        time_step=time_step,
        enable_visualization=True,
    )

    # Important: The cameras need the be created _after_ the simulation is
    # initialized.
    if args.config_dir:
        cameras = camera.create_trifinger_camera_array_from_config(
            args.config_dir, calib_filename_pattern=args.param_file_format
        )
    else:
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
            key = cv2.waitKey(int(time_step * 1000))
            if key == ord("q"):
                return


if __name__ == "__main__":
    main()
