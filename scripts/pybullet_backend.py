#!/usr/bin/env python3
"""Run robot_interfaces Backend for pyBullet using multi-process robot data."""
import argparse
import math

import robot_interfaces
from trifinger_simulation import (
    collision_objects,
    drivers,
    finger_types_data,
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--finger-type",
        choices=finger_types_data.get_valid_finger_types(),
        required=True,
        help="""Pass a valid finger type.""",
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
        "--max-number-of-actions",
        "-a",
        type=int,
        default=0,
        help="""Maximum numbers of actions that are processed.  After this the
            backend shuts down automatically.
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
        "--add-cube",
        action="store_true",
        help="""Spawn a cube and run the object tracker backend.""",
    )
    parser.add_argument(
        "--cameras",
        action="store_true",
        help="""Run camera backend using rendered images.""",
    )
    parser.add_argument(
        "--visualize",
        "-v",
        action="store_true",
        help="Run pyBullet's GUI for visualization.",
    )
    args = parser.parse_args()

    # select the correct types/functions based on which robot is used
    num_fingers = finger_types_data.get_number_of_fingers(args.finger_type)
    if num_fingers == 1:
        shared_memory_id = "finger"
        finger_types = robot_interfaces.finger
        create_backend = drivers.create_single_finger_backend
    elif num_fingers == 3:
        shared_memory_id = "trifinger"
        finger_types = robot_interfaces.trifinger
        create_backend = drivers.create_trifinger_backend

    # If max_number_of_actions is set, choose the history size of the time
    # series such that the whole episode fits in (+1 for the status message
    # containing the "limit exceeded" error).
    if args.max_number_of_actions:
        history_size = args.max_number_of_actions + 1
    else:
        history_size = 1000

    robot_data = finger_types.MultiProcessData(
        shared_memory_id, True, history_size=history_size
    )

    logger = finger_types.Logger(robot_data)

    backend = create_backend(
        robot_data,
        args.real_time_mode,
        args.visualize,
        args.first_action_timeout,
        args.max_number_of_actions,
    )
    backend.initialize()

    # Object and Object Tracker Interface
    # Important:  These objects need to be created _after_ the simulation is
    # initialized (i.e. after the SimFinger instance is created).
    if args.add_cube:
        # only import when really needed
        import trifinger_object_tracking.py_object_tracker as object_tracker

        # spawn a cube in the arena
        cube = collision_objects.Block()

        # initialize the object tracker interface
        object_tracker_data = object_tracker.Data("object_tracker", True)
        object_tracker_backend = object_tracker.SimulationBackend(  # noqa
            object_tracker_data, cube, args.real_time_mode
        )

    if args.cameras:
        from trifinger_cameras import tricamera

        camera_data = tricamera.MultiProcessData("tricamera", True, 10)
        camera_driver = tricamera.PyBulletTriCameraDriver()
        camera_backend = tricamera.Backend(camera_driver, camera_data)  # noqa

    backend.wait_until_terminated()

    if args.logfile:
        if args.max_number_of_actions:
            end_index = args.max_number_of_actions
        else:
            end_index = -1
        logger.write_current_buffer(
            args.logfile, start_index=0, end_index=end_index
        )


if __name__ == "__main__":
    main()
