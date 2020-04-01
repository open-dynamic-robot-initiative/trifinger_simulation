#!/usr/bin/env python3
"""Simple check if gym environment loads without error."""
import time
import gym


def main():
    # env = gym.make("pybullet_fingers.gym_wrapper:reach-v0", time_step=0.001,
    #               visual_debugging=True, finger_type="tri")
    env = gym.make(
        "pybullet_fingers.gym_wrapper:reachher-v0",
        time_step=0.001,
        visual_debugging=True,
        finger_type="tri",
    )

    time.sleep(10)
    env.reset()


if __name__ == "__main__":
    main()
