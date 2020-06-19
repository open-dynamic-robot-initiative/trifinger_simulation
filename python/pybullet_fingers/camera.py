import pybullet


class Camera(object):
    """Represents a camera in the simulation environment."""

    def __init__(
        self,
        camera_position,
        camera_orientation,
        image_size=(720, 540),
        pybullet_client=pybullet,
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
        self._pybullet_client = pybullet_client
        self._width = image_size[0]
        self._height = image_size[1]
        x = camera_orientation[0]
        y = camera_orientation[1]
        z = camera_orientation[2]
        w = camera_orientation[3]
        self._view_matrix = self._pybullet_client.computeViewMatrix(
            cameraEyePosition=camera_position,
            cameraTargetPosition=[
                2 * (x * z + w * y),
                2 * (y * z - w * x),
                1 - 2 * (x * x + y * y),
            ],
            cameraUpVector=[
                2 * (x * y - w * z),
                1 - 2 * (x * x + z * z),
                2 * (y * z + w * x),
            ],
        )

        self._proj_matrix = self._pybullet_client.computeProjectionMatrixFOV(
            fov=52,
            aspect=float(self._width) / self._height,
            nearVal=0.001,
            farVal=100.0,
        )

    def get_image(self):
        """Get a rendered image from the camera.

        Returns:
            (array, shape=(height, width, 3)):  Rendered RGB image from the
                simulated camera.
        """
        # FIXME: images are upside down
        (_, _, img, _, _) = self._pybullet_client.getCameraImage(
            width=self._width,
            height=self._height,
            viewMatrix=self._view_matrix,
            projectionMatrix=self._proj_matrix,
            renderer=pybullet.ER_BULLET_HARDWARE_OPENGL,
        )
        # remove the alpha channel
        return img[:, :, :3]


class TriFingerCameras:
    """Simulate the three cameras of the TriFinger platform."""

    def __init__(self):
        self.cameras = [
            # camera60
            Camera(
                camera_position=[0.2496, 0.2458, 0.4190],
                camera_orientation=[0.3760, 0.8690, -0.2918, -0.1354],
            ),
            # camera180
            Camera(
                camera_position=[0.0047, -0.2834, 0.4558],
                camera_orientation=[0.9655, -0.0098, -0.0065, -0.2603],
            ),
            # camera300
            Camera(
                camera_position=[-0.2470, 0.2513, 0.3943],
                camera_orientation=[-0.3633, 0.8686, -0.3141, 0.1220],
            ),
        ]

    def get_images(self):
        """Get images.

        Returns:
            List of images, one per camera.  Order is [camera60, camera180,
            camera300].  See Camera.get_image() for details.
        """
        return [c.get_image() for c in self.cameras]
