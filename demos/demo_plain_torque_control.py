#!/usr/bin/env python3
"""
To demonstrate reaching a randomly set target point in the arena using torque
control by directly specifying the position of the target only.
"""
import math
import time
import numpy as np

import ipdb

import robot_interfaces

from pybullet_fingers import sim_finger

if __name__ == "__main__":

    finger = sim_finger.Finger()


    last_time = None
    torque = np.array([0.5, 0., 0.0])
    
    for t in range(10000000):
        time.sleep(0.001)

        # current_time = time.time()
        # if not last_time is None:
        #     print('delta: ', current_time - last_time)
        # last_time = current_time

        if t % 100 == 0:
            torque = -torque
            
        if t > 100:
            torque = torque * 0
            
        action = robot_interfaces.finger.Action(torque=torque)
        t = finger.append_desired_action(action)
        observation = finger.get_observation(t)
        
        print(t)


    # finger.reset()
