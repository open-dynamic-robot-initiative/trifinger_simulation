#!/usr/bin/env python3
"""Simple demo on how to use the TriFingerPlatform interface."""
import argparse
import time

import cv2
import numpy as np

from trifinger_simulation import trifinger_platform, sample


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--enable-cameras",
        "-c",
        action="store_true",
        help="Enable camera observations.",
    )
    parser.add_argument(
        "--iterations",
        type=int,
        default=100,
        help="Number of motions that are performed.",
    )
    parser.add_argument(
        "--save-action-log",
        type=str,
        metavar="FILENAME",
        help="If set, save the action log to the specified file.",
    )
    parser.add_argument(
        "--object",
        type=str,
        choices=["cube", "dice", "none"],
        default="cube",
        metavar="OBJECT_TYPE",
        help="Which type of object to use (if any).",
    )
    args = parser.parse_args()

    object_type = trifinger_platform.ObjectType.NONE
    if args.object == "cube":
        object_type = trifinger_platform.ObjectType.COLORED_CUBE
    elif args.object == "dice":
        object_type = trifinger_platform.ObjectType.DICE

    platform = trifinger_platform.TriFingerPlatform(
        visualization=True,
        enable_cameras=args.enable_cameras,
        object_type=object_type,
    )

    # Move the fingers to random positions so that the cube is kicked around
    # (and thus it's position changes).
    for _ in range(args.iterations):
        goal = np.array(
            sample.random_joint_positions(
                number_of_fingers=3,
                lower_bounds=[-1, -1, -2],
                upper_bounds=[1, 1, 2],
            )
        )
        finger_action = platform.Action(position=goal)

        # apply action for a few steps, so the fingers can move to the target
        # position and stay there for a while
        for _ in range(250):
            t = platform.append_desired_action(finger_action)
            time.sleep(platform.get_time_step())

        # show the latest observations
        robot_observation = platform.get_robot_observation(t)
        print("Finger0 Position: %s" % robot_observation.position[:3])

        if args.object == "cube":
            camera_observation = platform.get_camera_observation(t)
            print(
                "Cube Position: %s"
                % camera_observation.filtered_object_pose.position
            )

        if platform.enable_cameras:
            for i, name in enumerate(("camera60", "camera180", "camera300")):
                # simulation provides images in RGB but OpenCV expects BGR
                img = cv2.cvtColor(
                    camera_observation.cameras[i].image, cv2.COLOR_RGB2BGR
                )
                cv2.imshow(name, img)
            cv2.waitKey(1)

        print()

    if args.save_action_log:
        platform.store_action_log(args.save_action_log)


if __name__ == "__main__":
    main()
