#!/usr/bin/env python3
"""view rendered and real images in parallel.

Loads robot and camera log files and plays back the images from the log while
at the same time rendering images in the simulation, using the robot and object
state from the log file.
"""
import argparse
import pathlib

import numpy as np
import cv2
import pybullet

import robot_fingers
from trifinger_cameras import utils
import trifinger_simulation
from trifinger_simulation import sim_finger, camera


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "log_dir", type=pathlib.Path, help="Path to the log files."
    )
    parser.add_argument(
        "--rate", type=int, default=1, help="Time in ms per step."
    )
    args = parser.parse_args()

    robot_log_file = str(args.log_dir / "robot_data.dat")
    camera_log_file = str(args.log_dir / "camera_data.dat")

    log = robot_fingers.TriFingerPlatformLog(robot_log_file, camera_log_file)

    simulation = sim_finger.SimFinger(
        finger_type="trifingerpro",
        enable_visualization=True,
    )
    cameras = camera.create_trifinger_camera_array_from_config(args.log_dir)

    cube_urdf_file = (
        pathlib.Path(trifinger_simulation.__file__).parent
        / "data/cube_v2/cube_v2.urdf"
    )
    cube = pybullet.loadURDF(
        fileName=str(cube_urdf_file),
    )
    assert cube >= 0, "Failed to load cube model."

    pybullet.configureDebugVisualizer(lightPosition=(0, 0, 1.0))
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_SHADOWS, 1)
    pybullet.configureDebugVisualizer(
        pybullet.COV_ENABLE_RGB_BUFFER_PREVIEW, 0
    )
    pybullet.configureDebugVisualizer(
        pybullet.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0
    )
    # yes, it is really a segmentation maRk...
    pybullet.configureDebugVisualizer(
        pybullet.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0
    )

    for t in range(
        log.get_first_timeindex(), log.get_last_timeindex() + 1, 100
    ):
        robot_observation = log.get_robot_observation(t)
        simulation.reset_finger_positions_and_velocities(
            robot_observation.position
        )

        # get rendered images from simulation
        sim_images = cameras.get_images()
        # images are rgb --> convert to bgr for opencv
        sim_images = [
            cv2.cvtColor(img, cv2.COLOR_RGB2BGR) for img in sim_images
        ]
        sim_images = np.hstack(sim_images)

        # get real images from cameras
        try:
            camera_observation = log.get_camera_observation(t)
            # set cube pose
            pybullet.resetBasePositionAndOrientation(
                cube,
                camera_observation.filtered_object_pose.position,
                camera_observation.filtered_object_pose.orientation,
            )

            real_images = [
                utils.convert_image(cam.image)
                for cam in camera_observation.cameras
            ]
            real_images = np.hstack(real_images)
        except Exception as e:
            print(e)
            real_images = np.zeros_like(sim_images)

        # display images
        image = np.vstack((sim_images, real_images))
        cv2.imshow("cameras", image)
        key = cv2.waitKey(args.rate)
        # exit on "q"
        if key == ord("q"):
            return


if __name__ == "__main__":
    main()
