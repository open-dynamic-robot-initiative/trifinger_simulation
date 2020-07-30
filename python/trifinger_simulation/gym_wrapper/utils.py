import numpy as np
import time
import datetime


def scale(x, space):
    """
    Scale some input to be between the range [-1;1] from the range
    of the space it belongs to
    """
    return 2.0 * (x - space.low) / (space.high - space.low) - 1.0


def unscale(y, space):
    """
    Unscale some input from [-1;1] to the range of another space
    """
    return space.low + (y + 1.0) / 2.0 * (space.high - space.low)


def compute_distance(a, b):
    """
    Returns the Euclidean distance between two
    vectors (lists/arrays)
    """
    return np.linalg.norm(np.subtract(a, b))


def sleep_until(until, accuracy=0.01):
    """
    Sleep until the given time.

    Args:
        until (datetime.datetime): Time until the function should sleep.
        accuracy (float): Accuracy with which it will check if the "until" time
            is reached.

    """
    while until > datetime.datetime.now():
        time.sleep(accuracy)
