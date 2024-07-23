"""Wrappers around Pinocchio for easy forward and inverse kinematics."""

import warnings

from robot_properties_fingers import Kinematics


warnings.warn(
    "The pinocchio_utils module has been moved to the robot_properties_fingers package."
    "  Please update your code accordingly.  This version in trifinger_simulation is"
    " deprecated and will be removed soon!",
    FutureWarning,
    stacklevel=1,
)

__all__ = ["Kinematics"]
