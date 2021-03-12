"""Unit tests for the camera module."""
import os
import pathlib
import pytest
import numpy as np

from trifinger_simulation import camera


TEST_DATA_DIR, _ = os.path.splitext(__file__)


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


def test_camera_parameters_load():
    config_file = os.path.join(TEST_DATA_DIR, "camera180_full.yml")

    expected_camera_matrix = np.array(
        [
            [585.06, 0.0, 369.13],
            [0.0, 587.96, 278.41],
            [0.0, 0.0, 1.0],
        ]
    )

    expected_dist_coeffs = np.array(
        [-0.2373, 0.1466, -0.0033, 0.0002, -0.1356]
    )

    expected_tf_mat = np.array(
        [
            [0.9999, 0.0011, 0.0009, 0.0015],
            [0.0015, -0.8603, -0.5096, 0.0059],
            [0.0001, 0.5096, -0.8603, 0.5336],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )

    with open(config_file, "r") as f:
        params = camera.CameraParameters.load(f)

    assert type(params) == camera.CameraParameters
    assert params.name == "camera180"
    assert params.width == 720
    assert params.height == 540
    np.testing.assert_array_almost_equal(
        params.camera_matrix, expected_camera_matrix
    )
    np.testing.assert_array_almost_equal(
        params.distortion_coefficients, expected_dist_coeffs
    )
    np.testing.assert_array_almost_equal(
        params.tf_world_to_camera, expected_tf_mat
    )


def test_load_camera_parameters():
    params = camera.load_camera_parameters(
        pathlib.Path(TEST_DATA_DIR), "camera{id}_full.yml"
    )

    # loading in general is already covered above, so only verify correct order
    # here
    assert len(params) == 3
    assert params[0].name == "camera60"
    assert params[1].name == "camera180"
    assert params[2].name == "camera300"
