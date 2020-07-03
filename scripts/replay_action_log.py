#!/usr/bin/env python3
"""Replay actions for a given logfile and verify final object pose.

The log file is a JSON file as produced by
:meth:`pybullet_fingers.TriFingerPlatform.store_action_log` which contains the
initial state, a list of all applied actions and the final state of the object.

The simulation is initialised according to the logged values and the actions
are applied one by one.  In the end, it is verified if the final object state
in the simulation matches the one in the log file.
"""
import argparse
import json
import numpy as np

from pybullet_fingers import trifinger_platform


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logfile", type=str, help="Path to the log file.")
    args = parser.parse_args()

    with open(args.logfile, "r") as fh:
        log = json.load(fh)

    initial_object_pose = (
        np.asarray(log["initial_object_pose"]["position"]),
        np.asarray(log["initial_object_pose"]["orientation"]),
    )
    platform = trifinger_platform.TriFingerPlatform(
        visualization=False, initial_object_pose=initial_object_pose
    )

    # TODO verify initial finger pose

    for logged_action in log["actions"]:
        action = platform.Action()
        action.torque = np.array(logged_action["torque"])
        action.position = np.array(logged_action["position"])
        action.position_kp = np.array(logged_action["position_kp"])
        action.position_kd = np.array(logged_action["position_kd"])

        t = platform.append_desired_action(action)

        assert logged_action["t"] == t

    cube_pose = platform.get_object_pose(t)
    final_pose = log["final_object_pose"]

    # print("Expected object pose: {} / {}".format(final_pose["position"],
    #                                             final_pose["orientation"]))
    # print("Actual object pose:   {} / {}".format(cube_pose.position,
    #                                             cube_pose.orientation))

    np.testing.assert_array_equal(cube_pose.position, final_pose["position"])
    np.testing.assert_array_equal(cube_pose.orientation, final_pose["orientation"])

    print("Passed.")


if __name__ == "__main__":
    main()
