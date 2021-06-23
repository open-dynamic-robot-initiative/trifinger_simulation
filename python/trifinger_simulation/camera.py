"""Simulated cameras for rendering images."""
import itertools
import typing
import pathlib

import yaml
import numpy as np
import pybullet
from scipy.spatial.transform import Rotation


def calib_data_to_matrix(data: dict) -> np.ndarray:
    """Extract a matrix from a camera parameter dict (as loaded from YAML)."""
    return np.array(data["data"]).reshape(data["rows"], data["cols"])


# TODO: this is more or less a duplicate of trifinger_cameras/CameraParameters
class CameraParameters(typing.NamedTuple):
    """Represents intrinsic and extrinsic parameters of a camera."""

    #: Name of the camera.
    name: str
    #: Width of the images.
    width: int
    #: Height of the images.
    height: int
    #: Camera projection matrix.  Shape = (3, 3)
    camera_matrix: np.ndarray
    #: Distortion coefficients.  Shape = (5,)
    distortion_coefficients: np.ndarray
    #: Transformation matrix from world to camera frame.  Shape = (4, 4)
    tf_world_to_camera: np.ndarray

    @classmethod
    def load(cls, stream: typing.TextIO) -> "CameraParameters":
        """Load camera parameters from a YAML stream."""
        data = yaml.safe_load(stream)

        name = data["camera_name"]
        width = data["image_width"]
        height = data["image_height"]
        camera_matrix = calib_data_to_matrix(data["camera_matrix"])
        dist_coeffs = calib_data_to_matrix(data["distortion_coefficients"])[0]
        tf_world_to_camera = calib_data_to_matrix(data["tf_world_to_camera"])

        return cls(
            name, width, height, camera_matrix, dist_coeffs, tf_world_to_camera
        )

    def dump(self, stream: typing.TextIO):
        """Dump camera parameters in YAML format to the given output stream."""
        # save all the data
        calibration_data = {
            "camera_name": self.name,
            # make sure width and height are plain strings (not numpy.int64 or
            # the like)
            "image_width": int(self.width),
            "image_height": int(self.height),
        }

        calibration_data["camera_matrix"] = {
            "rows": 3,
            "cols": 3,
            "data": self.camera_matrix.flatten().tolist(),
        }

        calibration_data["distortion_coefficients"] = {
            "rows": 1,
            "cols": 5,
            "data": self.distortion_coefficients.flatten().tolist(),
        }

        calibration_data["tf_world_to_camera"] = {
            "rows": 4,
            "cols": 4,
            "data": self.tf_world_to_camera.flatten().tolist(),
        }

        yaml.dump(
            calibration_data,
            stream,
            default_flow_style=False,
            sort_keys=False,
        )


class BaseCamera:
    def get_image(
        self, renderer=pybullet.ER_BULLET_HARDWARE_OPENGL
    ) -> np.ndarray:
        raise NotImplementedError()


class Camera(BaseCamera):
    """Represents a camera in the simulation environment.

    Note:  This class uses a simplified camera model.  For images that better
    match with the real cameras, use ``CalibratedCamera``.
    """

    def __init__(
        self,
        camera_position,
        camera_orientation,
        image_size=(270, 270),
        field_of_view=52,
        near_plane_distance=0.001,
        far_plane_distance=100.0,
        pybullet_client_id=0,
    ):
        """Initialize.

        Args:
            camera_position:  Position (x, y, z) of the camera w.r.t. the world
                frame.
            camera_orientation:  Quaternion (x, y, z, w) representing the
                orientation of the camera.
            image_size:  Tuple (width, height) specifying the size of the
                image.
            field_of_view: Field of view of the camera
            near_plane_distance: see OpenGL's documentation for details
            far_plane_distance: see OpenGL's documentation for details
            target_position: where should the camera be pointed at
            camera_up_vector: the up axis of the camera
            pybullet_client_id:  Id of the pybullet client (needed when
                multiple clients are running in parallel).
        """
        self._pybullet_client_id = pybullet_client_id
        self._width = image_size[0]
        self._height = image_size[1]

        camera_rot = Rotation.from_quat(camera_orientation)
        target_position = camera_rot.apply([0, 0, 1])
        camera_up_vector = camera_rot.apply([0, -1, 0])

        self._view_matrix = pybullet.computeViewMatrix(
            cameraEyePosition=camera_position,
            cameraTargetPosition=target_position,
            cameraUpVector=camera_up_vector,
            physicsClientId=self._pybullet_client_id,
        )

        self._proj_matrix = pybullet.computeProjectionMatrixFOV(
            fov=field_of_view,
            aspect=float(self._width) / self._height,
            nearVal=near_plane_distance,
            farVal=far_plane_distance,
            physicsClientId=self._pybullet_client_id,
        )

    def get_image(
        self, renderer=pybullet.ER_BULLET_HARDWARE_OPENGL
    ) -> np.ndarray:
        """Get a rendered image from the camera.

        Args:
            renderer: Specify which renderer is to be used. The renderer used
                by default relies on X server. Note: this would need
                visualization to have access to OpenGL. In order to use the
                renderer without visualization, as in, in the "DIRECT" mode of
                connection, use the ER_TINY_RENDERER.

        Returns:
            (array, shape=(height, width, 3)):  Rendered RGB image from the
                simulated camera.
        """
        (_, _, img, _, _) = pybullet.getCameraImage(
            width=self._width,
            height=self._height,
            viewMatrix=self._view_matrix,
            projectionMatrix=self._proj_matrix,
            renderer=renderer,
            physicsClientId=self._pybullet_client_id,
        )
        # remove the alpha channel
        return img[:, :, :3]


class CalibratedCamera(BaseCamera):
    r"""Simulate a camera based on calibration parameters.

    This class renders images from the simulation, using calibration parameters
    from a real camera.  It uses a more accurate projection matrix as
    ``Camera`` and also takes distortion into account.

    Args:
        camera_matrix:  Camera matrix containing focal length and centre point:

            .. math::
                \begin{bmatrix}
                  f_x &  0  & c_x \\
                   0  & f_y & c_y \\
                   0  &  0  &  0
                \end{matrix}

        distortion_coefficients:  Distortion coefficients
            ``(k_1, k_2, p_1, p_2, k_3)``
        tf_world_to_camera:  Homogeneous transformation matrix from world to
            camera frame.
        image_size:  Size of the image given as ``(width, height)``.
        near_plane_distance:  Minimum distance to camera for objects to be
            rendered.  Objects that are closer to the camera are clipped.
        far_plane_distance:  Maximum distance to the camera for objects to be
            rendered.  Objects that are further away are clipped.
        pybullet_client_id:  Id of the pybullet client (needed when multiple
            clients are running in parallel).
    """

    # How this class works internally
    # ===============================
    #
    # The "projection" and "view" matrices required by pyBullet are computed in
    # __init__ based on the given camera_matrix and tf_world_to_camera.
    #
    # The distortion cannot be directly integrated there, so it is applied
    # after rendering, using the given distortion_coefficients.
    #
    # When distorting the rendered image, there is the problem that the images
    # shrinks a bit, that is, when the size of the image should be preserved,
    # some pixels close to the edge will be empty.
    # To avoid this, some padding is added for the rendering, i.e.  the
    # rendered image is a bit larger then the desired output.  After
    # distortion, the padding is removed again, so that the resulting image has
    # the desired size.

    def __init__(
        self,
        camera_matrix,
        distortion_coefficients,
        tf_world_to_camera,
        image_size,
        near_plane_distance,
        far_plane_distance,
        pybullet_client_id=0,
    ):
        self._pybullet_client_id = pybullet_client_id

        #: Width of the output images.
        self._output_width = image_size[0]
        #: Height of the output images
        self._output_height = image_size[1]

        # Padding that is added to the rendered image
        self._padding = (
            round(self._output_width * 0.1),
            round(self._output_height * 0.1),
        )

        # size of the rendered (undistorted) image
        self._render_width = self._output_width + 2 * self._padding[0]
        self._render_height = self._output_height + 2 * self._padding[1]

        # adjust the centre point in the camera matrix to the padding
        center_offset = np.array(
            [[0, 0, self._padding[0]], [0, 0, self._padding[1]], [0, 0, 0]]
        )

        self._camera_matrix = camera_matrix + center_offset
        self._distortion_coefficients = distortion_coefficients

        # In OpenGL the camera is looking into negative z-direction, so we need
        # to rotate the camera pose by 180 degree around x.
        rot_x_180 = np.array(
            [
                [1, 0, 0, 0],
                [0, -1, 0, 0],
                [0, 0, -1, 0],
                [0, 0, 0, 1],
            ]
        )
        tf_mat = rot_x_180 @ tf_world_to_camera
        self._view_matrix = tf_mat.flatten(order="F")

        # pyBullet's computeProjectionMatrix() makes some simplifying
        # assumptions (f_x == f_y, c_x, c_y = image_size / 2) which are not
        # generally true.  Therefore compute the projection matrix manually.
        #
        # https://stackoverflow.com/a/60450420/2095383
        # https://www.songho.ca/opengl/gl_projectionmatrix.html
        # https://amytabb.com/ts/2019_06_28/

        # get focal length and centre point from camera matrix
        f_x = self._camera_matrix[0, 0]
        f_y = self._camera_matrix[1, 1]
        c_x = self._camera_matrix[0, 2]
        c_y = self._camera_matrix[1, 2]

        x_scale = 2 / self._render_width * f_x
        y_scale = 2 / self._render_height * f_y
        near = near_plane_distance
        far = far_plane_distance

        x_shift = 1 - 2 * c_x / self._render_width
        y_shift = (2 * c_y - self._render_height) / self._render_height
        proj_mat = np.array(
            [
                [x_scale, 0, x_shift, 0],
                [0, y_scale, y_shift, 0],
                [
                    0,
                    0,
                    (near + far) / (near - far),
                    2 * near * far / (near - far),
                ],
                [0, 0, -1, 0],
            ]
        )
        self._proj_matrix = proj_mat.flatten(order="F")

    def distort_image(self, image):
        """Distort an image based on the cameras distortion coefficients.

        Args:
            image:  The undistorted image.

        Returns:
            The distorted image.
        """

        # this function is based on the formulas from here:
        # https://stackoverflow.com/a/58131157/2095383
        # Computations are done on numpy arrays as much as possible for
        # performance reasons.

        distorted_image = np.zeros_like(image)

        f_x = self._camera_matrix[0, 0]
        f_y = self._camera_matrix[1, 1]
        c_x = self._camera_matrix[0, 2]
        c_y = self._camera_matrix[1, 2]
        k_1, k_2, p_1, p_2, k_3 = self._distortion_coefficients

        f = np.array([f_y, f_x])
        c = np.array([c_y, c_x])

        image_points = np.array(
            tuple(
                itertools.product(
                    range(self._render_height), range(self._render_width)
                )
            )
        )

        # normalize the image coordinates
        norm_points = (image_points - c) / f
        norm_points_square = norm_points ** 2
        norm_points_xy = norm_points.prod(axis=1)

        # determining the radial distortion
        r2 = np.sum(norm_points_square, axis=1)
        icdist = 1 / (1 - ((k_3 * r2 + k_2) * r2 + k_1) * r2)

        # determining the tangential distortion
        p = np.array([[p_2, p_1]])

        r2_plus_2_point_sq = r2[:, None] + 2 * norm_points_square
        delta = 2 * p * norm_points_xy[:, None] + p[::-1] * r2_plus_2_point_sq

        distorted_points = (norm_points + delta) * icdist[:, None]

        # un-normalise
        distorted_points = distorted_points * f + c

        # float to int
        distorted_points = distorted_points.round().astype(int)

        # filter out points that are outside the image
        in_image_idx = np.all(
            np.logical_and(
                (0, 0) <= distorted_points,
                distorted_points < (self._render_height, self._render_width),
            ),
            axis=1,
        )
        distorted_points = distorted_points[in_image_idx]
        image_points = image_points[in_image_idx]

        # finally construct the distorted image
        distorted_image[tuple(distorted_points.T)] = image[
            tuple(image_points.T)
        ]

        return distorted_image

    def get_image(
        self, renderer=pybullet.ER_BULLET_HARDWARE_OPENGL
    ) -> np.ndarray:
        """Get a rendered image from the camera.

        Args:
            renderer: Specify which renderer is to be used. The renderer used
                by default relies on X server. Note: this would need
                visualization to have access to OpenGL. In order to use the
                renderer without visualization, as in, in the "DIRECT" mode of
                connection, use the ER_TINY_RENDERER.

        Returns:
            array, shape=(height, width, 3):  Rendered RGB image from the
                simulated camera.
        """
        (_, _, img, _, _) = pybullet.getCameraImage(
            width=self._render_width,
            height=self._render_height,
            viewMatrix=self._view_matrix,
            projectionMatrix=self._proj_matrix,
            renderer=renderer,
            physicsClientId=self._pybullet_client_id,
        )
        # remove the alpha channel
        img = img[:, :, :3]

        # distort image
        img = self.distort_image(img)

        # remove the padding
        img = img[
            self._padding[1] : -self._padding[1],
            self._padding[0] : -self._padding[0],
        ]

        return img


class CameraArray:
    """Array of an arbitrary number of cameras.

    Args:
        cameras (Camera): List of cameras.
    """

    def __init__(self, cameras: typing.Sequence[BaseCamera]):
        self.cameras = cameras

    def get_images(
        self, renderer=pybullet.ER_BULLET_HARDWARE_OPENGL
    ) -> typing.List[np.ndarray]:
        """Get images.

        See Camera.get_image() for details.

        Returns:
            List of RGB images, one per camera.
        """
        return [c.get_image(renderer=renderer) for c in self.cameras]

    def get_bayer_images(
        self, renderer=pybullet.ER_BULLET_HARDWARE_OPENGL
    ) -> typing.List[np.ndarray]:
        """Get Bayer images.

        Same as get_images() but returning the images as BG-Bayer patterns
        instead of RGB.
        """
        return [rbg_to_bayer_bg(img) for img in self.get_images(renderer)]


def create_trifinger_camera_array(
    camera_parameters: typing.Iterable[CameraParameters],
    pybullet_client_id=0,
) -> CameraArray:
    """Create a TriFinger camera array using camera calibration parameters.

    Args:
        camera_parameters:  List of camera calibration parameters for the three
            cameras.
        pybullet_client_id:  Id of the pybullet client (needed when multiple
            clients are running in parallel).

    Returns:
        CameraArray with three cameras.
    """
    camera_ids = (60, 180, 300)

    cameras = []
    for camera_id, params in zip(camera_ids, camera_parameters):
        # Sanity check to verify the camera parameters are given in the correct
        # order.
        camera_name = "camera{}".format(camera_id)
        if params.name != camera_name:
            raise ValueError(
                "Expected parameters for camera {} but got {}".format(
                    camera_name, params.name
                )
            )

        image_size = (params.width, params.height)

        camera = CalibratedCamera(
            params.camera_matrix,
            params.distortion_coefficients,
            params.tf_world_to_camera,
            image_size=image_size,
            near_plane_distance=0.02,
            far_plane_distance=2.0,
            pybullet_client_id=pybullet_client_id,
        )
        cameras.append(camera)

    return CameraArray(cameras)


def load_camera_parameters(
    config_dir: pathlib.Path,
    filename_pattern: str = "camera{id}.yml",
) -> typing.Tuple[CameraParameters, ...]:
    camera_ids = (60, 180, 300)

    def load_params(id):
        with open(config_dir / filename_pattern.format(id=id)) as f:
            return CameraParameters.load(f)

    camera_parameters = tuple(load_params(id) for id in camera_ids)

    return camera_parameters


def create_trifinger_camera_array_from_config(
    config_dir: pathlib.Path,
    calib_filename_pattern="camera{id}.yml",
    pybullet_client_id=0,
) -> CameraArray:
    """Create a TriFinger camera array using camera calibration files.

    Loads camera calibration files from the given directory and uses them to
    create a :class:`CameraArray` of :class:`CalibratedCamera`s.

    Args:
        config_dir:  Directory containing the camera calibration files.
        calib_filename_pattern:  Template for the camera calibration file
            names.  '{id}' will be replaced with the camera id (60, 180, 300).
            Default: %(default)s
        pybullet_client_id:  Id of the pybullet client (needed when multiple
            clients are running in parallel).

    Returns:
        CameraArray with three cameras.
    """
    camera_parameters = load_camera_parameters(
        config_dir, calib_filename_pattern
    )

    return create_trifinger_camera_array(camera_parameters, pybullet_client_id)


class TriFingerCameras(CameraArray):
    """Simulate the three cameras of the TriFinger platform.

    Note: To get more accurate cameras (i.e. matching more closely the real
    cameras) use :func:`create_trifinger_camera_array_from_config` instead.
    """

    def __init__(self, **kwargs):
        cameras = [
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

        super().__init__(cameras)


def rbg_to_bayer_bg(image: np.ndarray) -> np.ndarray:
    """Convert an rgb image to a BG Bayer pattern.

    This can be used to generate simulated raw camera data in Bayer format.
    Note that there will be some loss in image quality.  It is mostly meant for
    testing the full software pipeline with the same conditions as on the real
    robot.  It is not optimized of realistic images.

    Args:
        image: RGB image.

    Returns:
        Bayer pattern based on the input image.  Height and width are the same
        as of the input image.  The image can be converted using OpenCV's
        `COLOR_BAYER_BG2*`.
    """
    # there is only one channel but it still needs the third dimension, so that
    # the conversion to a cv::Mat in C++ is easier
    bayer_img = np.zeros((image.shape[0], image.shape[1], 1), dtype=np.uint8)

    # channel names, assuming input is RGB
    CHANNEL_RED = 0
    CHANNEL_GREEN = 1
    CHANNEL_BLUE = 2

    # channel map to get the following pattern (called "BG" in OpenCV):
    #
    #   RG
    #   GB
    #
    channel_map = {
        (0, 0): CHANNEL_RED,
        (1, 0): CHANNEL_GREEN,
        (0, 1): CHANNEL_GREEN,
        (1, 1): CHANNEL_BLUE,
    }

    for r in range(image.shape[0]):
        for c in range(image.shape[1]):
            channel = channel_map[(r % 2, c % 2)]
            bayer_img[r, c] = image[r, c, channel]

    return bayer_img
