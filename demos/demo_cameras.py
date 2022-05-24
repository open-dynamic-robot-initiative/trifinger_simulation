#!/usr/bin/env python3
"""Demo showing how to add cameras in the TriFinger simulation.

Press "q" to exit.
Images can be saved by specifying an output directory with `--save-dir` and then
pressing "s" to save the current images.  Images will be named "camera{id}_##.png".
Existing images that match this pattern will be overwritten!
"""
import argparse
import pathlib
import typing

import numpy as np
import cv2

from trifinger_simulation import sim_finger, sample, camera


def save_images(
    images: typing.List[np.ndarray], output_dir: pathlib.Path, counter: int
):
    """Save images to the given directory, appending ``counter`` to the filenames.

    Files will be saved to ``output_dir`` with the pattern
    ``camera{id}_{counter:02d}.png``.
    Existing files that match this name will be overwritten!
    """
    for i, name in enumerate(("camera60", "camera180", "camera300")):
        filename = "{}_{:02d}.png".format(name, counter)
        outpath = output_dir / filename
        print("Save image {}".format(outpath))
        cv2.imwrite(str(outpath), images[i])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--robot",
        "-r",
        choices=["trifingerone", "trifingerpro"],
        default="trifingerpro",
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
    parser.add_argument(
        "--save-dir",
        type=pathlib.Path,
        help="Directory to which images are saved when pressing 's'.",
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

    save_counter = 0

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

            if key == ord("s"):
                if args.save_dir:
                    save_images(images, args.save_dir, save_counter)
                    save_counter += 1
                else:
                    print("ERROR: --save-dir not set, cannot save images.")
            elif key == ord("q"):
                return


if __name__ == "__main__":
    main()
