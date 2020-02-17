#!/usr/bin/env python3
"""Basic demo on how to control the Finger robot in reality and simulation.

This script illustrates how to control a robot via the Python interface.
"""
import os
import numpy as np
import rospkg

import robot_interfaces
import blmc_robots
from pybullet_fingers import sim_finger


def get_random_position():
    """Generate a random position within a save range."""
    position_min = np.array([-1, -1, -2])
    position_max = np.array([1, 1, 2])

    position_range = position_max - position_min

    return position_min + np.random.rand(3) * position_range


def demo_position_commands(finger):
    """Demo for directly sending position commands."""

    while True:
        # Run a position controller that randomly changes the desired position
        # every 300 steps.
        # One time step corresponds to roughly 1 ms.

        desired_position = get_random_position()
        for _ in range(300):
            # Appends a torque command ("action") to the action queue.
            # Returns the time step at which the action is going to be
            # executed.
            action = robot_interfaces.finger.Action(position=desired_position)
            t = finger.append_desired_action(action)
            observation = finger.get_observation(t)

        # print current position from time to time
        print("Position: %s" % observation.position)


def main():
    backend_type = "simulation"

    if backend_type == "real":
        # Use the default config file from the blmc_robots package
        config_file_path = os.path.join(
            rospkg.RosPack().get_path("blmc_robots"), "config", "finger.yml")

        # Storage for all observations, actions, etc.
        finger_data = robot_interfaces.finger.Data()

        # The backend sends actions from the data to the robot and writes
        # observations from the robot to the data.
        real_finger_backend = blmc_robots.create_real_finger_backend(
            finger_data, config_file_path)

        #fake_finger_backend = blmc_robots.create_fake_finger_backend(finger_data)

        # The frontend is used by the user to get observations and send actions
        frontend = robot_interfaces.finger.Frontend(finger_data)

        # Initializes the robot (e.g. performs homing).
        real_finger_backend.initialize()

    elif backend_type == "simulation":
        # action_bounds only need to be set when using SimFinger for sampling
        # possitions, which is not done in this demo.
        frontend = sim_finger.SimFinger(time_step=0.001,
                                        enable_visualization=True,
                                        finger_type="single",
                                        action_bounds=None)
        frontend.disable_velocity_control()

    else:
        raise ValueError("Invalid backend type")

    # move the robot
    demo_position_commands(frontend)


if __name__ == "__main__":
    main()
