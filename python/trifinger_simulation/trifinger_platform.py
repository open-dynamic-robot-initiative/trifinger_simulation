import json
import numpy as np

from .tasks import move_cube
from .sim_finger import SimFinger
from . import camera, collision_objects


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


class TriCameraObservation:
    """Pure-python copy of trifinger_cameras.tricamera.TriCameraObservation."""

    __slots__ = ["cameras"]

    def __init__(self):
        #: list of :class:`CameraObservation`: List of observations of cameras
        #: "camera60", "camera180" and "camera300" (in this order).
        self.cameras = [CameraObservation() for i in range(3)]


class TriFingerPlatform:
    """
    Wrapper around the simulation providing the same interface as
    ``robot_interfaces::TriFingerPlatformFrontend``.

    The following methods of the robot_interfaces counterpart are not
    supported:

    - get_robot_status()
    - wait_until_timeindex()

    """

    def __init__(
        self,
        visualization=False,
        initial_object_pose=None,
        enable_cameras=False,
    ):
        """Initialize.

        Args:
            visualization (bool):  Set to true to run visualization.
            initial_object_pose:  Initial pose for the manipulation object.
                Can be any object with attributes ``position`` (x, y, z) and
                ``orientation`` (x, y, z, w).  This is optional, if not set, a
                random pose will be sampled.
            enable_cameras (bool):  Set to true to enable camera observations.
                By default this is disabled as rendering of images takes a lot
                of computational power.  Therefore the cameras should only be
                enabled if the images are actually used.
        """
        #: Camera rate in frames per second.  Observations of camera and
        #: object pose will only be updated with this rate.
        #: NOTE: This is currently not used!
        self.camera_rate_fps = 30

        #: Set to true to render camera observations
        self.enable_cameras = enable_cameras

        #: Simulation time step
        self._time_step = 0.004

        # first camera update in the first step
        self._next_camera_update_step = 0

        # Initially move the fingers to a pose where they are guaranteed to not
        # collide with the object on the ground.
        initial_position = [0.0, np.deg2rad(-70), np.deg2rad(-130)] * 3

        self.simfinger = SimFinger(
            finger_type="trifingerpro",
            time_step=self._time_step,
            enable_visualization=visualization,
        )

        # set fingers to initial pose
        self.simfinger.reset_finger(initial_position)

        if initial_object_pose is None:
            initial_object_pose = move_cube.sample_goal(difficulty=-1)
        self.cube = collision_objects.Block(
            initial_object_pose.position, initial_object_pose.orientation
        )

        self._action_log = {
            "initial_robot_position": initial_position,
            "initial_object_pose": {
                "position": initial_object_pose.position.tolist(),
                "orientation": initial_object_pose.orientation.tolist(),
            },
            "actions": [],
        }

        self.tricamera = camera.TriFingerCameras()

        # forward "RobotFrontend" methods directly to simfinger
        self.Action = self.simfinger.Action
        self.get_desired_action = self.simfinger.get_desired_action
        self.get_applied_action = self.simfinger.get_applied_action
        self.get_timestamp_ms = self.simfinger.get_timestamp_ms
        self.get_current_timeindex = self.simfinger.get_current_timeindex
        self.get_robot_observation = self.simfinger.get_observation

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
        # next_t = self.get_current_timeindex() + 1
        # has_camera_update = next_t >= self._next_camera_update_step
        # if has_camera_update:
        #     self._next_camera_update_step += (
        #         self._compute_camera_update_step_interval()
        #     )

        #     self._object_pose_t = self._get_current_object_pose()
        #     if self.enable_cameras:
        #         self._camera_observation_t = (
        #             self._get_current_camera_observation()
        #         )

        has_camera_update = True
        self._object_pose_t = self._get_current_object_pose()
        if self.enable_cameras:
            self._camera_observation_t = self._get_current_camera_observation()

        t = self.simfinger.append_desired_action(action)

        # The correct timestamp can only be acquired now that t is given.
        # Update it accordingly in the object and camera observations
        if has_camera_update:
            camera_timestamp_s = self.get_timestamp_ms(t) / 1000
            self._object_pose_t.timestamp = camera_timestamp_s
            if self.enable_cameras:
                for i in range(len(self._camera_observation_t.cameras)):
                    self._camera_observation_t.cameras[
                        i
                    ].timestamp = camera_timestamp_s

        # write the desired action to the log
        self._action_log["actions"].append(
            {
                "t": t,
                "torque": action.torque.tolist(),
                "position": action.position.tolist(),
                "position_kp": action.position_kp.tolist(),
                "position_kd": action.position_kd.tolist(),
            }
        )

        return t

    def _get_current_object_pose(self, t=None):
        cube_state = self.cube.get_state()
        pose = ObjectPose()
        pose.position = np.asarray(cube_state[0])
        pose.orientation = np.asarray(cube_state[1])
        pose.confidence = 1.0
        # NOTE: The timestamp can only be set correctly after time step t is
        # actually reached.  Therefore, this is set to None here and filled
        # with the proper value later.
        if t is None:
            pose.timestamp = None
        else:
            pose.timestamp = self.get_timestamp_ms(t)

        return pose

    def get_object_pose(self, t):
        """Get object pose at time step t.

        Args:
            t:  The time index of the step for which the object pose is
                requested.  Only the value returned by the last call of
                :meth:`~append_desired_action` is valid.

        Returns:
            ObjectPose:  Pose of the object.  Values come directly from the
            simulation without adding noise, so the confidence is 1.0.

        Raises:
            ValueError: If invalid time index ``t`` is passed.
        """
        current_t = self.get_current_timeindex()

        if t == current_t:
            return self._object_pose_t
        elif t == current_t + 1:
            return self._get_current_object_pose(t)
        else:
            raise ValueError(
                "Given time index t has to match with index of the current"
                " step or the next one."
            )

    def _get_current_camera_observation(self, t=None):
        images = self.tricamera.get_images()
        observation = TriCameraObservation()
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

        return observation

    def get_camera_observation(self, t):
        """Get camera observation at time step t.

        Args:
            t:  The time index of the step for which the observation is
                requested.  Only the value returned by the last call of
                :meth:`~append_desired_action` is valid.

        Returns:
            TriCameraObservation:  Observations of the three cameras.  Images
            are rendered in the simulation.  Note that they are not optimized
            to look realistically.

        Raises:
            ValueError: If invalid time index ``t`` is passed.
        """
        if not self.enable_cameras:
            raise RuntimeError(
                "Cameras are not enabled.  Create `TriFingerPlatform` with"
                " `enable_cameras=True` if you want to use camera"
                " observations."
            )

        current_t = self.get_current_timeindex()

        if t == current_t:
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

        # TODO should the log also contain intermediate observations (object
        # and finger) for verification?

        t = self.simfinger.get_current_timeindex()
        object_pose = self.get_object_pose(t)
        self._action_log["final_object_pose"] = {
            "position": object_pose.position.tolist(),
            "orientation": object_pose.orientation.tolist(),
        }

        with open(filename, "w") as fh:
            json.dump(self._action_log, fh)
