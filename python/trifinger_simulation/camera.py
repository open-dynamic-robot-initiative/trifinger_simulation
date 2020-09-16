import typing

import numpy as np
import pybullet
from scipy.spatial.transform import Rotation


class Camera(object):
    """Represents a camera in the simulation environment."""

    def __init__(
        self,
        camera_position,
        camera_orientation,
        image_size=(270, 270),
        pybullet_client=pybullet,
        field_of_view=52,
        near_plane_distance=0.001,
        far_plane_distance=100.0,
        **kwargs,
    ):
        """Initialize.

        Args:
            camera_position:  Position (x, y, z) of the camera w.r.t. the world
                frame.
            camera_orientation:  Quaternion (x, y, z, w) representing the
                orientation of the camera.
            image_size:  Tuple (width, height) specifying the size of the
                image.
            pybullet_client:  Client for accessing the simulation.  By default
                the "pybullet" module is used directly.
        """
        self._kwargs = kwargs
        self._pybullet_client = pybullet_client
        self._width = image_size[0]
        self._height = image_size[1]

        camera_rot = Rotation.from_quat(camera_orientation)
        target_position = camera_rot.apply([0, 0, 1])
        camera_up_vector = camera_rot.apply([0, -1, 0])

        self._view_matrix = self._pybullet_client.computeViewMatrix(
            cameraEyePosition=camera_position,
            cameraTargetPosition=target_position,
            cameraUpVector=camera_up_vector,
            **self._kwargs,
        )

        self._proj_matrix = self._pybullet_client.computeProjectionMatrixFOV(
            fov=field_of_view,
            aspect=float(self._width) / self._height,
            nearVal=near_plane_distance,
            farVal=far_plane_distance,
            **self._kwargs,
        )

    def get_image(self, renderer=None) -> np.ndarray:
        """Get a rendered image from the camera.

        Returns:
            (array, shape=(height, width, 3)):  Rendered RGB image from the
                simulated camera.
        """
        if renderer is None:
            renderer=pybullet.ER_BULLET_HARDWARE_OPENGL
        (_, _, img, _, _) = self._pybullet_client.getCameraImage(
            width=self._width,
            height=self._height,
            viewMatrix=self._view_matrix,
            projectionMatrix=self._proj_matrix,
            renderer=renderer,
            **self._kwargs,
        )
        # remove the alpha channel
        return img[:, :, :3]


class TriFingerCameras:
    """Simulate the three cameras of the TriFinger platform."""

    def __init__(self, **kwargs):
        self.cameras = [
            # camera60
            Camera(
                camera_position=[0.2496, 0.2458, 0.4190],
                camera_orientation=[0.3760, 0.8690, -0.2918, -0.1354],
                **kwargs,
            ),
            # camera180
            Camera(
                camera_position=[0.0047, -0.2834, 0.4558],
                camera_orientation=[0.9655, -0.0098, -0.0065, -0.2603],
                **kwargs,
            ),
            # camera300
            Camera(
                camera_position=[-0.2470, 0.2513, 0.3943],
                camera_orientation=[-0.3633, 0.8686, -0.3141, 0.1220],
                **kwargs,
            ),
        ]

    def get_images(self) -> typing.List[np.ndarray]:
        """Get images.

        Returns:
            List of RGB images, one per camera.  Order is [camera60, camera180,
            camera300].  See Camera.get_image() for details.
        """
        return [c.get_image() for c in self.cameras]

    def get_bayer_images(self) -> typing.List[np.ndarray]:
        """Get Bayer images.

        Same as get_images() but returning the images as BG-Bayer patterns
        instead of RGB.
        """
        return [rbg_to_bayer_bg(c.get_image()) for c in self.cameras]


def rbg_to_bayer_bg(image: np.ndarray) -> np.ndarray:
    """Convert an rgb image to a BG Bayer pattern.

    This can be used to generate simulated raw camera data in Bayer format.
    Note that there will be some loss in image quality.  It is mostly meant for
    testing the full software pipeline with the same conditions as on the real
    robot.  It is not optimized of realistic images.

    Args:
        image: RGB image.

    Returns:
        BG-Bayer pattern based on the input image.  Height and width are the
        same as of the input image.
    """
    # there is only one channel but it still needs the third dimension, so that
    # the conversion to a cv::Mat in C++ is easier
    bayer_img = np.zeros((image.shape[0], image.shape[1], 1), dtype=np.uint8)

    # channel names, assuming input is RGB
    CHANNEL_RED = 0
    CHANNEL_GREEN = 1
    CHANNEL_BLUE = 2

    # channel map to get the following pattern:
    #
    #   BG
    #   GR
    #
    channel_map = {
        (0, 0): CHANNEL_BLUE,
        (1, 0): CHANNEL_GREEN,
        (0, 1): CHANNEL_GREEN,
        (1, 1): CHANNEL_RED,
    }

    for r in range(image.shape[0]):
        for c in range(image.shape[1]):
            channel = channel_map[(r % 2, c % 2)]
            bayer_img[r, c] = image[r, c, channel]

    return bayer_img
