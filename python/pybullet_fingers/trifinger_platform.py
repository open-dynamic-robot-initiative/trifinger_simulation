import numpy as np

from .sim_finger import SimFinger
from . import collision_objects  # , camera


class ObjectPose:
    """A pure-python copy of trifinger_object_tracking::ObjectPose."""
    __slots__ = ["position", "orientation", "timestamp", "confidence"]

    def __init__(self):
        self.position = np.zeros(3)
        self.orientation = np.zeros(4)
        self.timestamp = 0.0
        self.confidence = 1.0


class TriFingerPlatform:
    """
    Wrapper around the simulation providing the same interface as
    robot_interfaces::TriFingerPlatformFrontend.

    The following methods of the robot_interfaces counterpart are not
    supported:

    - get_robot_status()
    - wait_until_timeindex()

    """

    def __init__(self, visualization=False):
        """Initialize.

        Args:
            visualization (bool):  Set to true to run visualization.
        """
        self.simfinger = SimFinger(0.001, visualization, "tri")
        self.cube = collision_objects.Block()

        # TODO Add camera when corresponding PR is merged
        # self.tricamera = camera.TriFingerCameras()

        # forward "RobotFrontend" methods directly to simfinger
        self.Action = self.simfinger.Action
        self.append_desired_action = self.simfinger.append_desired_action
        self.get_desired_action = self.simfinger.get_desired_action
        self.get_applied_action = self.simfinger.get_applied_action
        self.get_timestamp_ms = self.simfinger.get_timestamp_ms
        self.get_current_timeindex = self.simfinger.get_current_timeindex
        self.get_robot_observation = self.simfinger.get_observation

    def get_object_pose(self, t):
        self.simfinger._validate_time_index(t)

        cube_state = self.cube.get_state()
        pose = ObjectPose()
        pose.position = np.asarray(cube_state[0])
        pose.position = np.asarray(cube_state[0])
        pose.timestamp = self.get_timestamp_ms(t) * 1000.0
        pose.confidence = 1.0

        return pose

    def get_camera_observation(self, t):
        raise NotImplementedError()
