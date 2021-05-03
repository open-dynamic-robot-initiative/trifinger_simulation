import argparse
import sys

import numpy as np
import cv2

import trifinger_simulation

from . import json_goal_from_config, sample_goal, goal_to_json
from . import visualize_2d, generate_goal_mask


def cmd_sample_goal(args):
    try:
        goal = sample_goal()
        print(goal_to_json(goal))
        if args.show:
            visualize_2d(goal)
        if args.show_masks:
            data_dir = trifinger_simulation.get_data_dir()
            camera_param_dir = data_dir / "camera_params"
            camera_params = trifinger_simulation.camera.load_camera_parameters(
                camera_param_dir, "camera{id}_cropped_and_downsampled.yml"
            )
            masks = generate_goal_mask(camera_params, goal)
            masks = np.hstack(masks)
            cv2.imshow("Goal Masks", masks)
            cv2.waitKey()

    except FileExistsError as e:
        print(e, file=sys.stderr)
        sys.exit(1)


def cmd_goal_from_config(args):
    try:
        goal = json_goal_from_config(args.goal_config_file)
        print(goal)
    except Exception as e:
        print(e, file=sys.stderr)
        sys.exit(1)


def main():
    parser = argparse.ArgumentParser("rearrange_dice", description=__doc__)
    subparsers = parser.add_subparsers(title="command", dest="command")
    subparsers.required = True

    sub = subparsers.add_parser(
        "sample_goal",
        description="""Sample a random goal.  The goal is written to stdout as
            JSON string.
        """,
    )
    sub.add_argument(
        "--show", action="store_true", help="Visualize the goal positions."
    )
    sub.add_argument(
        "--show-masks",
        action="store_true",
        help="Show the goal masks (using some default camera parameters).",
    )
    sub.set_defaults(func=cmd_sample_goal)

    sub = subparsers.add_parser(
        "goal_from_config",
        description="""Load or sample a goal based on the given config file.
            The goal is writtten to stdout as JSON string.
        """,
    )
    sub.add_argument(
        "goal_config_file", type=str, help="Path to a goal config JSON file."
    )
    sub.set_defaults(func=cmd_goal_from_config)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
