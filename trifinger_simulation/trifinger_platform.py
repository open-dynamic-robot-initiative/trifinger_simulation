import copy
import enum
import pickle
import numpy as np
import gymnasium as gym
from types import SimpleNamespace
import typing

from .tasks import move_cube, rearrange_dice
from .sim_finger import SimFinger, int_to_rgba
from . import camera, collision_objects, trifingerpro_limits


class ObjectType(enum.Enum):
    """Enumeration of supported object types."""

    NONE = 0
    COLORED_CUBE = 1
    DICE = 2


class ObjectPose:
    """A pure-python copy of trifinger_object_tracking::ObjectPose."""

    __slots__ = ["position", "orientation", "confidence"]

    def __init__(self):
        #: array: Position (x, y, z) of the object.  Units are meters.
        self.position = np.zeros(3)
        #: array: Orientation of the object as (x, y, z, w) quaternion.
        self.orientation = np.zeros(4)
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
    """Python version of trifinger_cameras::TriCameraObservation.

    This is a pure-python implementation of
    trifinger_cameras::TriCameraObservation, so we don't need to depend on
    trifinger_cameras here.
    """

    __slots__ = ["cameras"]

    def __init__(self):
        #: list of :class:`CameraObservation`: List of observations of cameras
        #: "camera60", "camera180" and "camera300" (in this order).
        self.cameras = [CameraObservation() for i in range(3)]


class TriCameraObjectObservation:
    """Python version of trifinger_object_tracking::TriCameraObjectObservation.

    This is a pure-python implementation of
    trifinger_object_tracking::TriCameraObjectObservation, so we don't need to
    depend on trifinger_object_tracking here.
    """

    __slots__ = ["cameras", "object_pose", "filtered_object_pose"]

    def __init__(self):
        #: list of :class:`CameraObservation`: List of observations of cameras
        #: "camera60", "camera180" and "camera300" (in this order).
        self.cameras = [CameraObservation() for i in range(3)]

        #: ObjectPose: Pose of the object in world coordinates.
        self.object_pose = ObjectPose()

        #: ObjectPose: Filtered pose of the object in world coordinates.  In
        #: simulation, this is the same as the unfiltered object_pose.
        self.filtered_object_pose = ObjectPose()


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
        low=spaces.robot_torque.low,
        high=spaces.robot_torque.high,
        dtype=np.float64,
    )
    spaces.robot_position.gym = gym.spaces.Box(
        low=spaces.robot_position.low,
        high=spaces.robot_position.high,
        dtype=np.float64,
    )
    spaces.robot_velocity.gym = gym.spaces.Box(
        low=spaces.robot_velocity.low,
        high=spaces.robot_velocity.high,
        dtype=np.float64,
    )
    spaces.object_position.gym = gym.spaces.Box(
        low=spaces.object_position.low,
        high=spaces.object_position.high,
        dtype=np.float64,
    )
    spaces.object_orientation.gym = gym.spaces.Box(
        low=spaces.object_orientation.low,
        high=spaces.object_orientation.high,
        dtype=np.float64,
    )

    def __init__(
        self,
        visualization: bool = False,
        initial_robot_position: typing.Optional[typing.Sequence[float]] = None,
        initial_object_pose=None,
        enable_cameras: bool = False,
        time_step_s: float = 0.001,
        object_type: ObjectType = ObjectType.COLORED_CUBE,
        camera_delay_steps: int = 90,  # default based on real robot data
    ):
        """Initialize.

        Args:
            visualization:  Set to true to run visualization.
            initial_robot_position: Initial robot joint angles
            initial_object_pose:  Only if ``object_type == COLORED_CUBE``:
                Initial pose for the manipulation object.  Can be any object
                with attributes ``position`` (x, y, z) and ``orientation`` (x,
                y, z, w).  If not set, a default pose is used.
            enable_cameras:  Set to true to enable rendered images in
                the camera observations.  If false camera observations are
                still available but the images will not be initialized.  By
                default this is disabled as rendering of images takes a lot of
                computational power.  Therefore the cameras should only be
                enabled if the images are actually used.
            time_step_s:  Simulation time step duration in seconds.
            object_type:  Which type of object to load.  This also influences
                some other aspects: When using the cube, the camera observation
                will contain an attribute ``object_pose``.
            camera_delay_steps:  Number of time steps by which camera
                observations are held back after they are generated.  This is
                used to simulate the delay of the camera observation that is
                happening on the real system due to processing (mostly the
                object detection).
        """
        #: Camera rate in frames per second.  Observations of camera and
        #: object pose will only be updated with this rate.
        self.camera_rate_fps = 10

        #: Set to true to render camera observations
        self.enable_cameras = enable_cameras

        #: Simulation time step
        self._time_step = time_step_s

        # Camera delay in robot time steps
        self._camera_delay_steps = camera_delay_steps

        # Time step at which the next camera update is triggered
        self._next_camera_trigger_t = 0
        # Time step at which the last triggered camera update is ready (used to
        # simulate delay).
        self._next_camera_observation_ready_t: typing.Optional[int] = None

        # Initialize robot, object and cameras
        # ====================================

        self.simfinger = SimFinger(
            finger_type="trifingerpro",
            time_step=self._time_step,
            enable_visualization=visualization,
        )

        if initial_robot_position is None:
            initial_robot_position = self.spaces.robot_position.default

        self.simfinger.reset_finger_positions_and_velocities(initial_robot_position)

        if initial_object_pose is None:
            initial_object_pose = move_cube.Pose(
                position=self.spaces.object_position.default,
                orientation=self.spaces.object_orientation.default,
            )

        self._has_object_tracking = False
        self.object_type = object_type
        if object_type == ObjectType.COLORED_CUBE:
            self.cube = collision_objects.ColoredCubeV2(
                position=initial_object_pose.position,
                orientation=initial_object_pose.orientation,
                pybullet_client_id=self.simfinger._pybullet_client_id,
            )
            self._has_object_tracking = True
        elif object_type == ObjectType.DICE:
            die_mass = 0.012
            # use a random goal for initial positions
            initial_positions = rearrange_dice.sample_goal()
            self.dice = [
                collision_objects.Cube(
                    position=pos,
                    half_width=rearrange_dice.DIE_WIDTH / 2,
                    mass=die_mass,
                    color_rgba=int_to_rgba(0x0A7DCF),
                )
                for pos in initial_positions
            ]

        self.tricamera = camera.TriFingerCameras(
            pybullet_client_id=self.simfinger._pybullet_client_id
        )

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
            "initial_robot_position": copy.copy(initial_robot_position),
            "initial_object_pose": copy.copy(initial_object_pose),
            "actions": [],
        }

        # get initial camera observation
        self._delayed_camera_observation = self._get_current_camera_observation(0)
        self._camera_observation_t = self._delayed_camera_observation

    def get_time_step(self):
        """Get simulation time step in seconds."""
        return self._time_step

    def _compute_camera_update_step_interval(self) -> int:
        return round((1.0 / self.camera_rate_fps) / self._time_step)

    def append_desired_action(self, action):
        """
        Call :meth:`pybullet.SimFinger.append_desired_action` and add the
        action to the action log.

        Arguments/return value are the same as for
        :meth:`pybullet.SimFinger.append_desired_action`.
        """
        camera_triggered = self._camera_update()

        t = self.simfinger.append_desired_action(action)

        # The correct timestamp can only be acquired now that t is given.
        # Update it accordingly in the camera observations
        if camera_triggered:
            camera_timestamp_s = self.get_timestamp_ms(t) / 1000
            for camera_ in self._delayed_camera_observation.cameras:
                camera_.timestamp = camera_timestamp_s

        # write the desired action to the log
        camera_obs = self.get_camera_observation(t)
        robot_obs = self.get_robot_observation(t)
        log_entry = {
            "t": t,
            "action": action,
            "robot_observation": robot_obs,
        }
        if self._has_object_tracking:
            log_entry["object_pose"] = camera_obs.object_pose
        # make a deep copy of log_entry to ensure all reference ties are cut
        self._action_log["actions"].append(copy.deepcopy(log_entry))

        return t

    def _camera_update(self) -> bool:
        # update camera and object observations only with the rate of the
        # cameras
        next_t = self.simfinger._t + 1

        # only trigger if no observation is still in work
        trigger_camera = (self._next_camera_observation_ready_t is None) and (
            next_t >= self._next_camera_trigger_t
        )

        if trigger_camera:
            self._delayed_camera_observation = self._get_current_camera_observation()
            self._next_camera_trigger_t += self._compute_camera_update_step_interval()
            self._next_camera_observation_ready_t = next_t + self._camera_delay_steps

        is_camera_observation_ready = (
            self._next_camera_observation_ready_t is not None
        ) and (next_t >= self._next_camera_observation_ready_t)

        if is_camera_observation_ready:
            self._camera_observation_t = self._delayed_camera_observation
            self._next_camera_observation_ready_t = None

        return trigger_camera

    def _get_current_object_pose(self):
        assert self._has_object_tracking

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

        if self._has_object_tracking:
            observation = TriCameraObjectObservation()
        else:
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

        if self._has_object_tracking:
            observation.object_pose = self._get_current_object_pose()
            observation.filtered_object_pose = self._get_current_object_pose()

        return observation

    def get_camera_observation(
        self, t: int
    ) -> typing.Union[TriCameraObservation, TriCameraObjectObservation]:
        """Get camera observation at time step t.

        .. important::

           Actual images are only rendered if the class was created with
           `enable_cameras=True` in the constructor, otherwise the images in
           the observation are set to None.  Other fields are still valid,
           though.

        Args:
            t:  The time index of the step for which the observation is
                requested.  Only the value returned by the last call of
                :meth:`~append_desired_action` is valid.

        Returns:
            Observations of the three cameras.  Depending of the object type
            used, this may also contain the object pose.

        Raises:
            ValueError: If invalid time index ``t`` is passed.
        """
        current_t = self.simfinger._t

        if t < 0:
            raise ValueError("Cannot access time index less than zero.")
        elif t == current_t or t == current_t + 1:
            return self._camera_observation_t
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

        if self._has_object_tracking:
            self._action_log["final_object_pose"] = {
                "t": t,
                "pose": camera_obs.object_pose,
            }

        with open(filename, "wb") as fh:
            pickle.dump(self._action_log, fh)
