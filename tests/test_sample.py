#!/usr/bin/env python3
import unittest
from numpy.testing import assert_array_compare
import operator

from trifinger_simulation import sample


def assert_array_less_equal(x, y, err_msg="", verbose=True):
    """Assert array x <= y.  Based on numpy.testing.assert_array_less."""
    assert_array_compare(
        operator.le,
        x,
        y,
        err_msg=err_msg,
        verbose=verbose,
        header="Arrays are not less-ordered",
        equal_inf=False,
    )


class TestSample(unittest.TestCase):
    """Test the functions of the sample module."""

    def test_random_point_positions(self):
        # choose bounds such that the ranges of the three joints are
        # non-overlapping
        lower_bounds = [-2, 0, 3]
        upper_bounds = [-1, 2, 5]

        # one finger
        for i in range(100):
            result = sample.random_joint_positions(
                1, lower_bounds, upper_bounds
            )
            assert_array_less_equal(lower_bounds, result)
            assert_array_less_equal(result, upper_bounds)

        # three finger
        for i in range(100):
            result = sample.random_joint_positions(
                3, lower_bounds, upper_bounds
            )
            assert_array_less_equal(lower_bounds * 3, result)
            assert_array_less_equal(result, upper_bounds * 3)


if __name__ == "__main__":
    unittest.main()
