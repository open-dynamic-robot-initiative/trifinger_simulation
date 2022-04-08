#!/usr/bin/env python3
import numpy as np


# TODO use named tuple for this?
class Observation:
    """Robot state observation."""

    #: Angular joint positions in radian.  Shape = (n_joints,)
    position: np.ndarray
    #: Joint velocities in rad/s.  Shape = (n_joints,)
    velocity: np.ndarray
    #: Joint torques in Nm.  Shape = (n_joints,)
    torque: np.ndarray
    #: Measurement of the push sensors on the finger tips.
    #: Shape = (n_fingers,)
    tip_force: np.ndarray

    def __init__(self):
        self.position = []
        self.velocity = []
        self.torque = []
        self.tip_force = []
