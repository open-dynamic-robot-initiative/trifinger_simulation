#!/usr/bin/env python3
import typing

import numpy as np


class Action:
    """Robot action."""

    #: Torque commands for the joints.
    torque: np.ndarray
    #: Position commands for the joints. Set to NaN to disable position control
    #: for the corresponding joint.
    position: np.ndarray
    #: P-gain for position controller.  Set to NaN to use default gain for the
    #: corresponding joint.
    position_kp: np.ndarray
    #: D-gain for position controller.  Set to NaN to use default gain for the
    #: corresponding joint.
    position_kd: np.ndarray

    def __init__(
        self,
        torque: typing.Sequence[float],
        position: typing.Sequence[float],
        kp: typing.Optional[np.ndarray] = None,
        kd: typing.Optional[np.ndarray] = None,
    ):
        """
        All parameters are expected to be of same length *N*, where *N* is the
        number of joints of the robot (e.g. 9 for TriFinger robots).

        Args:
            torque:  See :attr:`torque`.
            position:  See :attr:`position`.
            kp:  See :attr:`position_kp`.  If not set, default gains are used
                for all joints.
            kd:  See :attr:`position_kd`.  If not set, default gains are used
                for all joints.
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
