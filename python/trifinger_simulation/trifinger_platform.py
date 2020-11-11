import pickle
import warnings
import numpy as np
import gym
from types import SimpleNamespace

from .tasks import move_cube
from .sim_finger import SimFinger
from . import camera, collision_objects, trifingerpro_limits


class ObjectPose:
    """A pure-python copy of trifinger_object_tracking::ObjectPose."""

    __slots__ = ["position", "orientation", "timestamp", "confidence"]

    def __init__(self):
        #: array: Position (x, y, z) of the object.  Units are meters.
        self.position = np.zeros(3)
        #: array: Orientation of the object as (x, y, z, w) quaternion.
        self.orientation = np.zeros(4)
        #: float: Timestamp when the pose was observed.
        self.timestamp = 0.0
        #: float: Estimate of the confidence for this pose observation.
        self.confidence = 1.0


class CameraObservation:
    """Pure-python copy of trifinger_cameras.camera.CameraObservation."""

    __slots__ = ["image", "timestamp"]

    def __init__(self):
        #: array: The image.
        self.image = None
        #: float: Timestamp when the image was received.
        self.timestamp = None


class TriCameraObjectObservation:
    """Pure-python copy of trifinger_object_tracking::TriCameraObjectObservation."""

    __slots__ = ["cameras", "object_pose"]

    def __init__(self):
        #: list of :class:`CameraObservation`: List of observations of cameras
        #: "camera60", "camera180" and "camera300" (in this order).
        self.cameras = [CameraObservation() for i in range(3)]
        self.object_pose = ObjectPose()


class TriFingerPlatform:
    """
    Wrapper around the simulation providing the same interface as
    ``robot_interfaces::TriFingerPlatformFrontend``.

    The following methods of the robot_interfaces counterpart are not
    supported:

    - get_robot_status()
    - wait_until_timeindex()

    """

    # Create the action and observation spaces
    # ========================================

    spaces = SimpleNamespace()
    spaces.robot_torque = trifingerpro_limits.robot_torque
    spaces.robot_position = trifingerpro_limits.robot_position
    spaces.robot_velocity = trifingerpro_limits.robot_velocity
    spaces.object_position = trifingerpro_limits.object_position
    spaces.object_orientation = trifingerpro_limits.object_orientation

    # for convenience, we also create the respective gym spaces
    spaces.robot_torque.gym = gym.spaces.Box(
        low=spaces.robot_torque.low, high=spaces.robot_torque.high
    )
    spaces.robot_position.gym = gym.spaces.Box(
        low=spaces.robot_position.low, high=spaces.robot_position.high
    )
    spaces.robot_velocity.gym = gym.spaces.Box(
        low=spaces.robot_velocity.low, high=spaces.robot_velocity.high
    )
    spaces.object_position.gym = gym.spaces.Box(
        low=spaces.object_position.low, high=spaces.object_position.high
    )
    spaces.object_orientation.gym = gym.spaces.Box(
        low=spaces.object_orientation.low, high=spaces.object_orientation.high
    )

    def __init__(
        self,
        visualization=False,
        initial_robot_position=None,
        initial_object_pose=None,
        enable_cameras=False,
        time_step_s=0.004,
    ):
        """Initialize.

        Args:
            visualization (bool):  Set to true to run visualization.
            initial_robot_position: Initial robot joint angles
            initial_object_pose:  Initial pose for the manipulation object.
                Can be any object with attributes ``position`` (x, y, z) and
                ``orientation`` (x, y, z, w).  This is optional, if not set, a
                random pose will be sampled.
            enable_cameras (bool):  Set to true to enable rendered images in
                the camera observations.  If false camera observations are
                still available but the images will not be initialized.  By
                default this is disabled as rendering of images takes a lot of
                computational power.  Therefore the cameras should only be
                enabled if the images are actually used.
            time_step_s (float):  Simulation time step duration in seconds.
        """
        #: Camera rate in frames per second.  Observations of camera and
        #: object pose will only be updated with this rate.
        self.camera_rate_fps = 10

        #: Set to true to render camera observations
        self.enable_cameras = enable_cameras

        #: Simulation time step
        self._time_step = time_step_s

        # first camera update in the first step
        self._next_camera_update_step = 0

        # Initialize robot, object and cameras
        # ====================================

        self.simfinger = SimFinger(
            finger_type="trifingerpro",
            time_step=self._time_step,
            enable_visualization=visualization,
        )

        _kwargs = {"physicsClientId": self.simfinger._pybullet_client_id}

        if initial_robot_position is None:
            initial_robot_position = self.spaces.robot_position.default

        self.simfinger.reset_finger_positions_and_velocities(
            initial_robot_position
        )

        if initial_object_pose is None:
            initial_object_pose = move_cube.Pose(
                position=self.spaces.object_position.default,
                orientation=self.spaces.object_orientation.default,
            )

        self.cube = collision_objects.Cuboid(
            position=initial_object_pose.position,
            orientation=initial_object_pose.orientation,
            half_extents=[0.01, 0.04, 0.01],
            mass=0.016,
            pybullet_client_id=self.simfinger._pybullet_client_id,
        )

        self.tricamera = camera.TriFingerCameras(**_kwargs)

        # Forward some methods for convenience
        # ====================================
        # forward "RobotFrontend" methods directly to simfinger
        self.Action = self.simfinger.Action
        self.get_desired_action = self.simfinger.get_desired_action
        self.get_applied_action = self.simfinger.get_applied_action
        self.get_timestamp_ms = self.simfinger.get_timestamp_ms
        self.get_current_timeindex = self.simfinger.get_current_timeindex
        self.get_robot_observation = self.simfinger.get_observation

        # forward kinematics directly to simfinger
        self.forward_kinematics = self.simfinger.kinematics.forward_kinematics

        # Initialize log
        # ==============
        self._action_log = {
            "initial_robot_position": initial_robot_position,
            "initial_object_pose": initial_object_pose,
            "actions": [],
        }

    def get_time_step(self):
        """Get simulation time step in seconds."""
        return self._time_step

    def _compute_camera_update_step_interval(self):
        return (1.0 / self.camera_rate_fps) / self._time_step

    def append_desired_action(self, action):
        """
        Call :meth:`pybullet.SimFinger.append_desired_action` and add the
        action to the action log.

        Arguments/return value are the same as for
        :meth:`pybullet.SimFinger.append_desired_action`.
        """
        # update camera and object observations only with the rate of the
        # cameras
        next_t = self.simfinger._t + 1
        has_camera_update = next_t >= self._next_camera_update_step
        if has_camera_update:
            self._next_camera_update_step += (
                self._compute_camera_update_step_interval()
            )
            self._camera_observation_t = self._get_current_camera_observation()

        t = self.simfinger.append_desired_action(action)

        # The correct timestamp can only be acquired now that t is given.
        # Update it accordingly in the object and camera observations
        if has_camera_update:
            camera_timestamp_s = self.get_timestamp_ms(t) / 1000
            for i in range(len(self._camera_observation_t.cameras)):
                self._camera_observation_t.cameras[
                    i
                ].timestamp = camera_timestamp_s
            self._camera_observation_t.object_pose.timestamp = (
                camera_timestamp_s
            )

        # write the desired action to the log
        camera_obs = self.get_camera_observation(t)
        robot_obs = self.get_robot_observation(t)
        self._action_log["actions"].append(
            {
                "t": t,
                "action": action,
                "object_pose": camera_obs.object_pose,
                "robot_observation": robot_obs,
            }
        )

        return t

    def _get_current_object_pose(self):
        cube_state = self.cube.get_state()
        pose = ObjectPose()
        pose.position = np.asarray(cube_state[0])
        pose.orientation = np.asarray(cube_state[1])
        pose.confidence = 1.0

        return pose

    def _get_current_camera_observation(self, t=None):
        if self.enable_cameras:
            images = self.tricamera.get_images()
        else:
            images = [None] * 3

        observation = TriCameraObjectObservation()
        # NOTE: The timestamp can only be set correctly after time step t
        # is actually reached.  Therefore, this is set to None here and
        # filled with the proper value later.
        if t is None:
            timestamp = None
        else:
            timestamp = self.get_timestamp_ms(t)

        for i, image in enumerate(images):
            observation.cameras[i].image = image
            observation.cameras[i].timestamp = timestamp

        observation.object_pose = self._get_current_object_pose()

        return observation

    def get_camera_observation(self, t):
        """Get camera observation at time step t.

        Args:
            t:  The time index of the step for which the observation is
                requested.  Only the value returned by the last call of
                :meth:`~append_desired_action` is valid.

        Returns:
            TriCameraObjectObservation:  Observations of the three cameras.
            This also contains the object pose.
            Actual images are only rendered if the class was created with
            `enable_cameras=True` in the constructor, otherwise the images in
            the observation are set to None.  Other fields are still
            valid, though.

        Raises:
            ValueError: If invalid time index ``t`` is passed.
        """
        if not self.enable_cameras:
            warnings.warn(
                "Cameras are not enabled, so images in the camera observation"
                " are not initialized.  Create `TriFingerPlatform` with"
                " `enable_cameras=True` to get rendered camera images."
            )

        current_t = self.simfinger._t

        if t < 0:
            raise ValueError("Cannot access time index less than zero.")
        elif t == current_t:
            return self._camera_observation_t
        elif t == current_t + 1:
            return self._get_current_camera_observation(t)
        else:
            raise ValueError(
                "Given time index t has to match with index of the current"
                " step or the next one."
            )

    def store_action_log(self, filename):
        """Store the action log to a JSON file.

        Args:
            filename (str):  Path to the JSON file to which the log shall be
                written.  If the file exists already, it will be overwritten.
        """
        t = self.get_current_timeindex()
        camera_obs = self.get_camera_observation(t)
        self._action_log["final_object_pose"] = {
            "t": t,
            "pose": camera_obs.object_pose,
        }

        with open(filename, "wb") as fh:
            pickle.dump(self._action_log, fh)
