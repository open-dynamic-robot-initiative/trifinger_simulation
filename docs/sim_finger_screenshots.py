#!/usr/bin/env python3
"""Set up PyBullet for making screenshots with the different finger types."""
import argparse
import time
import typing
import pybullet

from trifinger_simulation import (
    collision_objects,
    finger_types_data,
    sim_finger,
)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "finger_type",
        choices=finger_types_data.get_valid_finger_types(),
    )
    args = parser.parse_args()

    time_step = 0.001

    finger = sim_finger.SimFinger(
        finger_type=args.finger_type,
        time_step=time_step,
        enable_visualization=True,
    )

    _config: typing.Dict[str, typing.Dict[str, typing.Sequence[float]]] = {
        "fingerone": {
            "joint_pos": (0, -0.7, -1.5),
            "cube_pos": (0.05, -0.08, 0.04),
        },
        "trifingerone": {
            "joint_pos": (0, -0.7, -1.5) * 3,
            "cube_pos": (0.05, 0.08, 0.04),
        },
        "fingeredu": {
            "joint_pos": (0, 0.7, -1.5),
            "cube_pos": (0.08, 0.05, 0.04),
        },
        "trifingeredu": {
            "joint_pos": (0, 0.7, -1.5) * 3,
            "cube_pos": (0.05, 0.08, 0.04),
        },
        "fingerpro": {
            "joint_pos": (0, 0.9, -1.7),
            "cube_pos": (0.05, -0.08, 0.04),
        },
        "trifingerpro": {
            "joint_pos": (0, 0.9, -1.7) * 3,
            "cube_pos": (0.05, -0.02, 0.04),
        },
    }
    config = _config[args.finger_type]

    camera_yaw = 100.0
    if args.finger_type == "fingeredu":
        # for FingerEdu rotate the camera by 90 deg as the finger is oriented
        # differently
        camera_yaw += 90.0

    # disable all gui elements and specify camera pose (so we have the same
    # fixed view in all screenshots)
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, 0)
    pybullet.resetDebugVisualizerCamera(
        cameraDistance=1.0,
        cameraYaw=camera_yaw,
        cameraPitch=-30.0,
        cameraTargetPosition=(0, 0, 0.2),
    )
    from scipy.spatial.transform import Rotation

    cube = collision_objects.ColoredCubeV2(
        position=config["cube_pos"],
        # rotate the cube a bit, so it looks nicer
        orientation=Rotation.from_euler(
            "xz", (90, -40), degrees=True
        ).as_quat(),
    )

    # set the finger position
    finger.reset_finger_positions_and_velocities(config["joint_pos"])

    while True:
        time.sleep(1)
