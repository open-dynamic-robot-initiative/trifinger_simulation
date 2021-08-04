#!/usr/bin/env python3
"""Load a set of images and a goal pattern and compute the reward."""
import argparse
import json
import pathlib
import os
import sys

import cv2
import numpy as np

from trifinger_object_tracking.py_lightblue_segmenter import segment_image

import trifinger_simulation.camera
from trifinger_simulation.tasks import rearrange_dice
from trifinger_simulation.tasks.rearrange_dice import utils


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--image-dir",
        type=pathlib.PurePath,
        required=True,
        help="Directory that contains images 'camera{id}.png'",
    )
    parser.add_argument(
        "--camera-calib-dir",
        type=pathlib.PurePath,
        help="""Directory containing camera calibration files.  Defaults to
            '{image_dir}/..'.
        """,
    )
    parser.add_argument(
        "--camera-calib-files",
        type=str,
        default="camera{id}_cropped_and_downsampled.yml",
        help="""Name pattern of the camera calibration files.  Use '{id}' as
            placeholder for the id.  Default: %(default)s.
        """,
    )
    parser.add_argument(
        "--goal-file",
        type=pathlib.PurePath,
        help="""Either a JSON file containing the goal or a txt file containing
            an ASCII goal pattern (will be distinguished by the file
            extension).  Defaults to '{image_dir}/goal.txt'.
        """,
    )
    args = parser.parse_args()

    camera_ids = (60, 180, 300)

    image_dir = args.image_dir
    camera_calib_dir = args.camera_calib_dir or image_dir / ".."
    camera_calib_file = args.camera_calib_files
    goal_file = args.goal_file or image_dir / "goal.txt"

    # load images
    images = [
        cv2.imread(os.fspath(image_dir / f"camera{id}.png"))
        for id in camera_ids
    ]

    # load camera parameters
    camera_info = trifinger_simulation.camera.load_camera_parameters(
        camera_calib_dir, camera_calib_file
    )

    # load goal
    with open(goal_file, "r") as f:
        if goal_file.suffix == ".json":
            goal = json.load(f)
        else:
            goal = utils.parse_pattern_file(f)

    goal_masks = rearrange_dice.generate_goal_mask(camera_info, goal)
    segmented_images = [segment_image(img) for img in images]

    reward = rearrange_dice.evaluate_state(goal_masks, segmented_images)
    print("Reward: -%f" % reward)

    # Overlay goal mask with segmentation and mark pixels red/green depending
    # on if they are inside or outside the goal area.
    labelled_segmentations = []
    for goalmask, segmask in zip(goal_masks, segmented_images):
        good = np.logical_and(goalmask, segmask)
        bad = np.logical_xor(good, segmask)
        good_but_empty = np.logical_and(goalmask, np.logical_not(segmask))

        labelled = np.zeros_like(images[0])
        labelled[good] = (0, 255, 0)
        labelled[bad] = (0, 0, 255)
        labelled[good_but_empty] = (0, 90, 0)

        labelled_segmentations.append(labelled)

    # show images
    images = np.hstack(images)
    labelled_segmentations = np.hstack(labelled_segmentations)
    debug_img = np.vstack([images, labelled_segmentations])

    cv2.imshow("image", debug_img)
    cv2.waitKey()

    return 0


if __name__ == "__main__":
    sys.exit(main())
