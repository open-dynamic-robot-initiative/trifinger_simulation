import argparse
import sys

import numpy as np
import cv2
import json

import trifinger_simulation

from . import json_goal_from_config, sample_goal, goal_to_json
from . import visualize_2d, generate_goal_mask
from . import utils


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


def cmd_create_pattern_template(args):
    print(utils.create_empty_pattern())


def cmd_parse_goal_pattern(args):
    try:
        goal = utils.parse_pattern_file(args.infile)
        print(json.dumps(goal))
        visualize_2d(goal)
    # except Exception as e:
    except IndentationError as e:
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

    sub = subparsers.add_parser(
        "create_pattern_template",
        description="Output template for an ASCII goal pattern.",
    )
    sub.set_defaults(func=cmd_create_pattern_template)

    sub = subparsers.add_parser(
        "parse_goal_pattern", description="Parse ASCII goal pattern."
    )
    sub.add_argument(
        "infile",
        type=argparse.FileType("r"),
        default="-",
        help="File with the goal pattern (use '-' to read from stdin).",
    )
    sub.set_defaults(func=cmd_parse_goal_pattern)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
