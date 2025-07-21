#!/usr/bin/env python3
"""Visualise the robot at the specified position with axes, etc. disabled.

This can be used to more easily create screenshots of the robot in specific poses.
"""
import argparse
import time
from trifinger_simulation import (
    sim_finger,
    finger_types_data,
)

import pybullet


def main():
    valid_finger_types = finger_types_data.get_valid_finger_types()

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "finger_type",
        choices=valid_finger_types,
        help="Specify a valid finger type.  One of {}".format(valid_finger_types),
    )
    parser.add_argument(
        "--position", "-p", type=float, nargs="+", help="Joint positions."
    )
    parser.add_argument(
        "--robot-height-offset",
        type=float,
        default=0,
        help="Offset to the default robot height.",
    )
    args = parser.parse_args()

    finger = sim_finger.SimFinger(
        finger_type=args.finger_type,
        enable_visualization=True,
        robot_position_offset=[0, 0, args.robot_height_offset],
    )

    # disable all GUI elements
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, 0)

    # change initial camera pose
    # pybullet.resetDebugVisualizerCamera(
    #     cameraDistance=1.0,
    #     cameraYaw=100.0,
    #     cameraPitch=-30.0,
    #     cameraTargetPosition=(0, 0, 0.2),
    #     physicsClientId=self._pybullet_client_id,
    # )

    if args.position:
        print("Joint positions: {}".format(args.position))
        finger.reset_finger_positions_and_velocities(args.position)

    while True:
        time.sleep(100)


if __name__ == "__main__":
    main()
