import argparse
import sys

from . import move_cube


def move_cube_argparse(parser):
    sub = parser.add_subparsers(title="command", dest="command")
    sub.required = True

    validate_file = sub.add_parser("validate_goal_file")
    validate_file.add_argument("filename", type=str)
    validate_file.set_defaults(func=move_cube_validate_goal_file)

    validate_file = sub.add_parser("sample_goal")
    validate_file.add_argument("difficulty", type=int, choices=[1, 2, 3, 4])
    validate_file.set_defaults(func=move_cube_sample_goal)


def move_cube_validate_goal_file(args):
    try:
        move_cube.validate_goal_file(args.filename)
    except Exception as e:
        print(e)
        sys.exit(1)


def move_cube_sample_goal(args):
    try:
        print(move_cube.sample_goal(args.difficulty).to_json())
    except Exception as e:
        print(e)
        sys.exit(1)


def main():
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(title="task", dest="task")
    subparsers.required = True

    parser_move_cube = subparsers.add_parser("move_cube")
    move_cube_argparse(parser_move_cube)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
