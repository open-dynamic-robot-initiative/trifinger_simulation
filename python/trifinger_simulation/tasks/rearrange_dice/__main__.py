import argparse
import sys

from . import json_goal_from_config, sample_goal, goal_to_json

from . import visualize_2d


def cmd_sample_goal(args):
    try:
        goal = sample_goal()
        print(goal_to_json(goal))
        if args.show:
            visualize_2d(goal)
    except Exception as e:
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
    sub.add_argument("--show", action="store_true", help="Visualize the goal.")
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
