#!/usr/bin/env python3
"""Demo for trying to solve the "move_cube" task in simulation."""
import time
import numpy as np

from pybullet_fingers.tasks import move_cube
from pybullet_fingers import trifinger_platform, sample


def main():
    # Create simulation
    platform = trifinger_platform.TriFingerPlatform(visualization=True)

    # sample a simple goal pose
    goal_position, goal_orientation = move_cube.sample_goal(difficulty=1)

    # Now move the fingers randomly for a while, hoping that they kick the cube
    # towards the goal :).

    # Stop when max. number of episodes is reached
    t = -1
    while t < move_cube.episode_length - 1:
        # sample random position action
        finger_action = platform.Action(position=np.array(
            sample.random_joint_positions(
                number_of_fingers=3,
                lower_bounds=[-1, -1, -2],
                upper_bounds=[1, 1, 2],
            )
        ))

        # apply the action for a few steps to give the fingers time to reach
        # there
        for _ in range(250):
            t = platform.append_desired_action(finger_action)
            # add sleep so visualization runs in real time
            time.sleep(0.001)

        # get current pose of the cube and compute the cost (smaller is better)
        cube_pose = platform.get_object_pose(t)
        cost = move_cube.evaluate_state(
            goal_position,
            goal_orientation,
            cube_pose.position,
            cube_pose.orientation,
        )
        print("t = {}, cost: {:.4f}".format(t, cost))

    print("==> Finished with cost {:.4f}\n".format(cost))


if __name__ == "__main__":
    main()
