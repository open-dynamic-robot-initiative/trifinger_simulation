#!/usr/bin/env python3


class Observation:
    """
    Robot state observation.

    The length of the attributes depends on the robot type.  In the following
    ``n_joints`` is the number of joints and ``n_fingers`` the number of
    fingers (e.g. for the TriFinger robots ``n_fingers = 3, n_joints = 9``).

    Attributes:
        position (array, shape=(n_joints,)):  Angular joint positions in radian.
        velocity (array, shape=(n_joints,)):  Joint velocities in rad/s.
        torque (array, shape=(n_joints,)):  Joint torques in Nm.
        tip_force (array, shape=(n_fingers,)):  Measurement of the push sensors
            on the finger tips.
    """

    def __init__(self):
        self.position = []
        self.velocity = []
        self.torque = []
        self.tip_force = []
