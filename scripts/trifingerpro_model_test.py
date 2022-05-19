#!/usr/bin/env python3
"""Script for testing the TriFingerPro model."""
import time
import pybullet
from trifinger_simulation import (
    sim_finger,
    visual_objects,
)


def visualize_collisions(sim):
    contact_points = pybullet.getContactPoints(
        bodyA=sim.finger_id,
        physicsClientId=sim._pybullet_client_id,
    )

    markers = []

    for cp in contact_points:
        contact_distance = cp[8]
        if contact_distance < 0:
            position = cp[5]
            marker = visual_objects.CubeMarker(
                width=0.01,
                position=position,
                orientation=(0, 0, 0, 1),
                color=(1, 0, 0, 1),
            )
            markers.append(marker)

    return markers


def main():
    # argparser = argparse.ArgumentParser(description=__doc__)
    # args = argparser.parse_args()

    time_step = 0.001

    finger = sim_finger.SimFinger(
        finger_type="trifingerpro",
        time_step=time_step,
        enable_visualization=True,
    )

    markers = []
    while True:
        # action = finger.Action(torque=[0.3, 0.3, -0.2] * 3)
        action = finger.Action(position=[0.0, 0.9, -1.7] * 3)
        t = finger.append_desired_action(action)
        finger.get_observation(t)

        # delete old markers
        for m in markers:
            del m
        markers = visualize_collisions(finger)

        time.sleep(time_step)


if __name__ == "__main__":
    main()
