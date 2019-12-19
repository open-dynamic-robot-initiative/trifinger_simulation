#!/usr/bin/env python3

class obstruct:
    """
    Defines the observation structure to be followed by the observations derived from sim_finger.
    """
    def __init__(self):
        position = []
        velocity = []
        torque = []

        self.position = position
        self.velocity = velocity
        self.torque = torque
