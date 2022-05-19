"""Run evaluation of the move_cube task.

For more information on the different commands run ``<command> --help``.
"""
import argparse
import sys

from . import json_goal_from_config, sample_goal, goal_to_json
from . import run_evaluate_policy
from . import run_replay
from . import replay_action_log


def cmd_sample_goal(args):
    try:
        t = sample_goal(args.difficulty)
        print(goal_to_json(t))
    except Exception as e:
        print(e, file=sys.stderr)
        sys.exit(1)


def cmd_goal_from_config(args):
    try:
        t = json_goal_from_config(args.goal_config_file)
        print(t)
    except Exception as e:
        print(e, file=sys.stderr)
        sys.exit(1)


def cmd_evaluate_policy(args):
    try:
        run_evaluate_policy.main(args.output_directory)
    except Exception as e:
        print(e, file=sys.stderr)
        sys.exit(1)


def cmd_replay_all(args):
    try:
        run_replay.main(args.input_directory)
    except Exception as e:
        print(e, file=sys.stderr)
        sys.exit(1)


def cmd_replay_one(args):
    try:
        replay_action_log.replay_action_log(
            args.logfile, args.difficulty, args.initial_pose, args.goal_pose
        )
    except Exception as e:
        print(e, file=sys.stderr)
        sys.exit(1)


def cmd_evaluate_and_replay(args):
    try:
        print()
        print("Run Policy on Random Goals")
        print("==========================")
        run_evaluate_policy.main(args.output_directory)

        print()
        print("Validate Logs and Compute Reward")
        print("================================")
        print()
        run_replay.main(args.output_directory)
    except Exception as e:
        print(e, file=sys.stderr)
        sys.exit(1)


def main():
    parser = argparse.ArgumentParser("move_cube", description=__doc__)
    subparsers = parser.add_subparsers(title="command", dest="command")
    subparsers.required = True

    sub = subparsers.add_parser(
        "sample_goal",
        description="""Sample a goal.  The trajectory is printed to
            stdout as JSON string.
        """,
    )
    sub.add_argument(
        "--difficulty",
        type=int,
        choices=[1, 2, 3, 4],
        default=4,
        help="Difficulty level of the goal.  Default: %(default)s.",
    )
    sub.set_defaults(func=cmd_sample_goal)

    sub = subparsers.add_parser(
        "goal_from_config",
        description="""Load or sample a goal trajectory based on the given
            config file.  The trajectory is printed to stdout as JSON string.
        """,
    )
    sub.add_argument(
        "goal_config_file", type=str, help="Path to a goal config JSON file."
    )
    sub.set_defaults(func=cmd_goal_from_config)

    sub = subparsers.add_parser(
        "evaluate_policy",
        description="Run a evaluate_policy script on a set of random goals.",
    )
    run_evaluate_policy.add_arguments(sub)
    sub.set_defaults(func=cmd_evaluate_policy)

    sub = subparsers.add_parser(
        "replay_all", description="Replay all logs in the given directory."
    )
    run_replay.add_arguments(sub)
    sub.set_defaults(func=cmd_replay_all)

    sub = subparsers.add_parser(
        "replay_one",
        description="""Replay an evaluation log file and compute reward based
            on the given goal.
        """,
    )
    replay_action_log.add_arguments(sub)
    sub.set_defaults(func=cmd_replay_one)

    sub = subparsers.add_parser(
        "evaluate_and_check",
        description="Run 'evaluate_policy' followed by 'replay_all'.",
    )
    run_evaluate_policy.add_arguments(sub)
    sub.set_defaults(func=cmd_evaluate_and_replay)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
