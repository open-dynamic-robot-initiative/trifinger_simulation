#!/usr/bin/env python3
import numpy as np


class Action:
    """Robot action.

    The length of the attributes depends on the robot type.  In the following
    ``n_joints`` is the number of joints and ``n_fingers`` the number of
    fingers (e.g. for the TriFinger robots ``n_fingers = 3, n_joints = 9``).

    Attributes:
        torque (array, shape=(n_joints,)):  Torque commands for the joints.
        position (array, shape=(n_joints,)):  Position commands for the joints.
            Set to NaN to disable position control for the corresponding joint.
        kp (array, shape=(n_joints,)):  P-gain for position controller.  Set to
            NaN to use default gain for the corresponding joint.
        kd (array, shape=(n_joints,)):  D-gain for position controller.  Set to
            NaN to use default gain for the corresponding joint.
    """

    def __init__(self, torque, position, kp=None, kd=None):
        """Initialize

        Args:
            torque:  See :attr:`torque`.
            position:  See :attr:`position`.
            kp:  See :attr:`kp`.
            kd:  See :attr:`kd`.
        """
        self.torque = np.asarray(torque)
        self.position = np.asarray(position)

        if kp is None:
            self.position_kp = np.full_like(position, np.nan, dtype=float)
        else:
            self.position_kp = kp

        if kd is None:
            self.position_kd = np.full_like(position, np.nan, dtype=float)
        else:
            self.position_kd = kd
