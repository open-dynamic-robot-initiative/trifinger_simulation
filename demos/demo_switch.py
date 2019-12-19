#!/usr/bin/env python3

"""
To demonstrate that the same commands can be used with both the real system and
the simulated environment.
"""

import math
import time
import numpy as np

from pybullet_fingers import sim_finger
import robot_interfaces
import blmc_robots
import matplotlib.pyplot as plt

def real_or_sim(which):

    Action = robot_interfaces.finger.Action

    upper_link_torques_to_plot = []
    middle_link_torques_to_plot = []
    lower_link_torques_to_plot = []

    if which == "real":
        fake_finger_data = robot_interfaces.finger.Data()
        fake_finger_backend = blmc_robots.create_fake_finger_backend(
            fake_finger_data)
        fake_finger = robot_interfaces.finger.Frontend(fake_finger_data)
        fake_finger_backend.initialize()
        model = fake_finger
        desired_torques = [0] * 3

    elif which == "simulated":
        simulated_finger = sim_finger.Finger()
        simulated_finger.disable_default_motors()
        model = simulated_finger

    for i in range(3):
        if which == "simulated":
            desired_positon = simulated_finger.set_target_in_arena_randomly()
            simulated_finger.display_object_at_target(desired_positon)

        for _ in range(2000):
            if which == "simulated":
                joint_positions = simulated_finger.inverse_kinematics(
                    desired_positon)
                desired_torques = simulated_finger.compute_pd_control_torques(joint_positions)
                applied_torques = simulated_finger._safety_torque_check(desired_torques)
                upper_link_torques_to_plot.append(applied_torques[0])
                middle_link_torques_to_plot.append(applied_torques[1])
                lower_link_torques_to_plot.append(applied_torques[2])

            action = robot_interfaces.finger.Action(torque=applied_torques)

            time_index = model.append_desired_action(
            action)

            if which == "simulated":
                simulated_finger._step_simulation()
                time.sleep(0.001)

            observation = model.get_observation(time_index)
            print(observation.position)

        i = i + 1
    print("max values now")
    print(max(upper_link_torques_to_plot), max(middle_link_torques_to_plot), max(lower_link_torques_to_plot))

    plt.plot(range(6000), upper_link_torques_to_plot)
    plt.plot(range(6000), middle_link_torques_to_plot)
    plt.plot(range(6000), lower_link_torques_to_plot)

    plt.show()

if __name__ == "__main__":
    # real_or_sim("real")
    real_or_sim("simulated")
