"""Unit tests for the camera module."""
import os
import pathlib
import numpy as np
import pybullet

from trifinger_simulation import camera as sim_camera


TEST_DATA_DIR, _ = os.path.splitext(__file__)


def test_calib_data_to_matrix_1x1():
    data = {
        "rows": 1,
        "cols": 1,
        "data": [42],
    }
    expected = np.array([[42]])

    np.testing.assert_array_equal(
        sim_camera.calib_data_to_matrix(data), expected
    )


def test_calib_data_to_matrix_3x3():
    data = {
        "rows": 3,
        "cols": 3,
        "data": [1, 2, 3, 4, 5, 6, 7, 8, 9],
    }
    expected = np.array([[1, 2, 3], [4, 5, 6], [7, 8, 9]])

    np.testing.assert_array_equal(
        sim_camera.calib_data_to_matrix(data), expected
    )


def test_calib_data_to_matrix_2x4():
    data = {
        "rows": 2,
        "cols": 4,
        "data": [1, 2, 3, 4, 5, 6, 7, 8],
    }
    expected = np.array([[1, 2, 3, 4], [5, 6, 7, 8]])

    np.testing.assert_array_equal(
        sim_camera.calib_data_to_matrix(data), expected
    )


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
        params = sim_camera.CameraParameters.load(f)

    assert type(params) == sim_camera.CameraParameters
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


def test_camera_parameters_dump(tmpdir):
    # load the test data
    config_file = os.path.join(TEST_DATA_DIR, "camera180_full.yml")
    with open(config_file, "r") as f:
        params = sim_camera.CameraParameters.load(f)

    # dump to a temporary file, load that again and verify that the values are
    # the same
    tmpfile = tmpdir.join("dump.yml")
    with open(tmpfile, "w") as f:
        params.dump(f)

    with open(tmpfile, "r") as f:
        params2 = sim_camera.CameraParameters.load(f)

    assert params.name == params2.name
    assert params.width == params2.width
    assert params.height == params2.height
    np.testing.assert_array_almost_equal(
        params.camera_matrix, params2.camera_matrix
    )
    np.testing.assert_array_almost_equal(
        params.distortion_coefficients, params2.distortion_coefficients
    )
    np.testing.assert_array_almost_equal(
        params.tf_world_to_camera, params2.tf_world_to_camera
    )


def test_load_camera_parameters():
    params = sim_camera.load_camera_parameters(
        pathlib.Path(TEST_DATA_DIR), "camera{id}_full.yml"
    )

    # loading in general is already covered above, so only verify correct order
    # here
    assert len(params) == 3
    assert params[0].name == "camera60"
    assert params[1].name == "camera180"
    assert params[2].name == "camera300"


def test_camera():
    pybullet.connect(pybullet.DIRECT)

    width = 150
    height = 100

    camera = sim_camera.Camera(
        (0, 0, 0.3),
        (0, 0, 0, 1),
        image_size=(width, height),
    )

    # only some very basic test, verifying that we get an image of the correct
    # shape
    image = camera.get_image()
    assert image.shape == (height, width, 3)

    pybullet.disconnect()


def test_calibrated_camera():
    config_file = os.path.join(TEST_DATA_DIR, "camera180_full.yml")

    pybullet.connect(pybullet.DIRECT)

    with open(config_file) as f:
        params = sim_camera.CameraParameters.load(f)
    camera = sim_camera.CalibratedCamera(
        params.camera_matrix,
        params.distortion_coefficients,
        params.tf_world_to_camera,
        (params.width, params.height),
        near_plane_distance=0.02,
        far_plane_distance=1.0,
    )

    # only some very basic test, verifying that we get an image of the correct
    # shape
    image = camera.get_image()
    assert image.shape == (params.height, params.width, 3)

    pybullet.disconnect()


def test_trifingercameras():
    pybullet.connect(pybullet.DIRECT)

    width = 150
    height = 100
    expected_shape = (height, width, 3)

    tricamera = sim_camera.TriFingerCameras(
        image_size=(width, height),
    )

    images = tricamera.get_images()
    assert len(images) == 3
    assert images[0].shape == expected_shape
    assert images[1].shape == expected_shape
    assert images[2].shape == expected_shape

    pybullet.disconnect()


def test_create_trifinger_camera_array_from_config():
    pybullet.connect(pybullet.DIRECT)

    expected_shape = (540, 720, 3)

    tricamera = sim_camera.create_trifinger_camera_array_from_config(
        pathlib.Path(TEST_DATA_DIR),
        "camera{id}_full.yml",
    )

    images = tricamera.get_images()
    assert len(images) == 3
    assert images[0].shape == expected_shape
    assert images[1].shape == expected_shape
    assert images[2].shape == expected_shape

    pybullet.disconnect()
