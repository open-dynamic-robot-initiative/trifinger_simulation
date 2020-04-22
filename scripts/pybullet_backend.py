#!/usr/bin/env python3
"""Run robot_interfaces Backend for pyBullet using multi-process robot data."""
import argparse
import math

import robot_interfaces
import pybullet_fingers.drivers


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--finger-type",
        choices=["single", "tri"],
        required=True,
        help="""Specify whether the Single Finger ("single")
                        or the TriFinger ("tri") is used.
                        """,
    )
    parser.add_argument(
        "--real-time-mode",
        "-r",
        action="store_true",
        help="""Run simulation in real time.  If not set, the simulation runs
            as fast as possible.
        """,
    )
    parser.add_argument(
        "--first-action-timeout",
        "-t",
        type=float,
        default=math.inf,
        help="""Timeout (in seconds) for reception of first action after
            starting the backend.  If not set, the timeout is disabled.
        """,
    )
    parser.add_argument(
        "--logfile",
        "-l",
        type=str,
        help="""Path to a file to which the data log is written.  If not
            specified, no log is generated.
        """,
    )
    parser.add_argument(
        "--visualize",
        action="store_true",
        help="Run pyBullet's GUI for visualization.",
    )
    args = parser.parse_args()

    # select the correct types/functions based on which robot is used
    if args.finger_type == "single":
        shared_memory_id = "finger"
        finger_types = robot_interfaces.finger
        create_backend = pybullet_fingers.drivers.create_single_finger_backend
    else:
        shared_memory_id = "trifinger"
        finger_types = robot_interfaces.trifinger
        create_backend = pybullet_fingers.drivers.create_trifinger_backend

    robot_data = finger_types.MultiProcessData(shared_memory_id, True)

    if args.logfile:
        logger = finger_types.Logger(robot_data, 100)
        logger.start(args.logfile)

    backend = create_backend(robot_data, args.real_time_mode, args.visualize,
                             args.first_action_timeout)
    backend.initialize()

    backend.wait_until_terminated()


if __name__ == "__main__":
    main()
