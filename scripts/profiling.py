#!/usr/bin/env python3
import cProfile
import pstats

import numpy as np

from trifinger_simulation.sim_finger import SimFinger
from trifinger_simulation import collision_objects


finger_names = ["trifingerone", "trifingeredu", "trifingerpro"]


def execute_random_motion(*, finger_name, nb_timesteps, enable_visualization):
    finger = SimFinger(
        finger_type=finger_name,
        enable_visualization=enable_visualization,
    )
    cube = collision_objects.Block()

    for t in range(nb_timesteps):
        if t % 50 == 0:
            torque = np.random.uniform(low=-0.36, high=0.36, size=(9))
        finger.append_desired_action(finger.Action(torque=torque))
    del finger
    del cube


if __name__ == "__main__":

    def get_filename(*, finger_name, enable_visualization):
        return "stats_" + finger_name + "_vis-" + str(enable_visualization)

    for enable_visualization in [False, True]:
        for finger_name in finger_names:

            def execute_random_motion_finger():
                execute_random_motion(
                    finger_name=finger_name,
                    nb_timesteps=1000,
                    enable_visualization=enable_visualization,
                )

            filename = get_filename(
                finger_name=finger_name,
                enable_visualization=enable_visualization,
            )
            cProfile.run("execute_random_motion_finger()", filename=filename)

    for enable_visualization in [False, True]:
        for finger_name in finger_names:
            filename = get_filename(
                finger_name=finger_name,
                enable_visualization=enable_visualization,
            )
            p = pstats.Stats(filename)
            print(filename, "==============================================")
            p.strip_dirs().sort_stats("tottime").print_stats(3)
