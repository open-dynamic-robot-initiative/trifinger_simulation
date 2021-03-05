"""Unit tests for the camera module."""
import os
import pytest
import numpy as np
import yaml

from trifinger_simulation import camera


@pytest.mark.calib_data_to_matrix
def test_calib_data_to_matrix_1x1():
    data = {
        "rows": 1,
        "cols": 1,
        "data": [42],
    }
    expected = np.array([[42]])

    np.testing.assert_array_equal(camera.calib_data_to_matrix(data), expected)


@pytest.mark.calib_data_to_matrix
def test_calib_data_to_matrix_3x3():
    data = {
        "rows": 3,
        "cols": 3,
        "data": [1, 2, 3, 4, 5, 6, 7, 8, 9],
    }
    expected = np.array([[1, 2, 3], [4, 5, 6], [7, 8, 9]])

    np.testing.assert_array_equal(camera.calib_data_to_matrix(data), expected)


@pytest.mark.calib_data_to_matrix
def test_calib_data_to_matrix_2x4():
    data = {
        "rows": 2,
        "cols": 4,
        "data": [1, 2, 3, 4, 5, 6, 7, 8],
    }
    expected = np.array([[1, 2, 3, 4], [5, 6, 7, 8]])

    np.testing.assert_array_equal(camera.calib_data_to_matrix(data), expected)
