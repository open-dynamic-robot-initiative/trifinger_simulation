"""Functions for sampling, validating and evaluating "move cube" goals."""
import json

import numpy as np
from scipy.spatial.transform import Rotation


#: Random number generator.  Replace with a seeded version for deterministic
#: samples.
random = np.random.RandomState()


#: Number of time steps in one episode (3750 steps per 0.004s = 15s)
episode_length = 3750


_CUBE_WIDTH = 0.065
_ARENA_RADIUS = 0.195

_cube_3d_radius = _CUBE_WIDTH * np.sqrt(3) / 2
_max_cube_com_distance_to_center = _ARENA_RADIUS - _cube_3d_radius

_min_height = _CUBE_WIDTH / 2
_max_height = 0.1


_cube_corners = np.array(
    [
        [-1, -1, -1],
        [-1, -1, +1],
        [-1, +1, -1],
        [-1, +1, +1],
        [+1, -1, -1],
        [+1, -1, +1],
        [+1, +1, -1],
        [+1, +1, +1],
    ]
) * (_CUBE_WIDTH / 2)


class InvalidGoalError(Exception):
    """Exception used to indicate that the given goal is invalid."""

    def __init__(self, message, position, orientation):
        super().__init__(message)
        self.position = position
        self.orientation = orientation


class Pose:
    """Represents a pose given by position and orientation."""

    def __init__(
        self, position=np.array([0, 0, 0]), orientation=np.array([0, 0, 0, 1])
    ):
        """Initialize.

        Args:
            position (numpy.ndarray): Position (x, y, z)
            orientation (numpy.ndarray): Orientation as quaternion (x, y, z, w)

        """
        #: Position (x, y, z).
        self.position = position
        #: Orientation as quaternion (x, y, z, w)
        self.orientation = orientation

    def to_dict(self):
        """Convert to dictionary."""
        return {"position": self.positions, "orientation": self.orientation}

    def to_json(self):
        """Convert to JSON string."""
        return goal_to_json(self)

    @classmethod
    def from_dict(cls, dict):
        """Create Pose instance from dictionary."""
        return cls(dict["position"], dict["orientation"])

    @classmethod
    def from_json(cls, json_str):
        """Create Pose instance from JSON string."""
        return goal_from_json(json_str)


def get_cube_corner_positions(pose):
    """Get the positions of the cube's corners with the given pose.

    Args:
        pose (Pose):  Pose of the cube.

    Returns:
        (array, shape=(8, 3)): Positions of the corners of the cube in the
            given pose.
    """
    rotation = Rotation.from_quat(pose.orientation)
    translation = np.asarray(pose.position)

    return rotation.apply(_cube_corners) + translation


def sample_goal(difficulty):
    """Sample a goal pose for the cube.

    Args:
        difficulty (int):  Difficulty level.  The higher, the more difficult is
            the goal.  Possible levels are:

            - 1: Random goal position on the table, no orientation.
            - 2: Fixed goal position in the air with x,y = 0.  No orientation.
            - 3: Random goal position in the air, no orientation.
            - 4: Random goal pose in the air, including orientation.

    Returns:
        (Pose): Goal pose of the cube relative to the world frame.  Note that
            the pose always contains an orientation.  For difficulty levels
            where the orientation is not considered, this is set to
            ``[0, 0, 0, 1]`` and will be ignored when computing the reward.
    """
    # difficulty -1 is for initialization

    def random_xy():
        # sample uniform position in circle (https://stackoverflow.com/a/50746409)
        radius = _max_cube_com_distance_to_center * np.sqrt(random.random())
        theta = random.uniform(0, 2 * np.pi)

        # x,y-position of the cube
        x = radius * np.cos(theta)
        y = radius * np.sin(theta)

        return x, y

    def random_yaw_orientation():
        yaw = random.uniform(0, 2 * np.pi)
        orientation = Rotation.from_euler("z", yaw)
        return orientation.as_quat()

    if difficulty == -1:  # for initialization
        # on the ground, random yaw
        x, y = random_xy()
        z = _CUBE_WIDTH / 2
        orientation = random_yaw_orientation()

    elif difficulty == 1:
        x, y = random_xy()
        z = _CUBE_WIDTH / 2
        orientation = np.array([0, 0, 0, 1])

    elif difficulty == 2:
        x = 0.0
        y = 0.0
        z = _min_height + 0.05
        orientation = np.array([0, 0, 0, 1])

    elif difficulty == 3:
        x, y = random_xy()
        z = random.uniform(_min_height, _max_height)
        orientation = np.array([0, 0, 0, 1])

    elif difficulty == 4:
        x, y = random_xy()
        # Set minimum height such that the cube does not intersect with the
        # ground in any orientation
        z = random.uniform(_cube_3d_radius, _max_height)
        orientation = Rotation.random(random_state=random).as_quat()

    else:
        raise ValueError("Invalid difficulty %d" % difficulty)

    goal = Pose()
    goal.position = np.array((x, y, z))
    goal.orientation = orientation

    return goal


def validate_goal(goal):
    """Validate that the given pose is a valid goal (e.g. no collision)

    Raises an error if the given goal pose is invalid.

    Args:
        goal (Pose):  Goal pose.

    Raises:
        ValueError:  If given values are not a valid 3d position/orientation.
        InvalidGoalError:  If the given pose exceeds the allowed goal space.
    """
    if len(goal.position) != 3:
        raise ValueError("len(goal.position) != 3")
    if len(goal.orientation) != 4:
        raise ValueError("len(goal.orientation) != 4")
    if np.linalg.norm(goal.position[:2]) > _max_cube_com_distance_to_center:
        raise InvalidGoalError(
            "Position is outside of the arena circle.",
            goal.position,
            goal.orientation,
        )
    if goal.position[2] < _min_height:
        raise InvalidGoalError(
            "Position is too low.", goal.position, goal.orientation
        )
    if goal.position[2] > _max_height:
        raise InvalidGoalError(
            "Position is too high.", goal.position, goal.orientation
        )

    # even if the CoM is above _min_height, a corner could be intersecting with
    # the bottom depending on the orientation
    corners = get_cube_corner_positions(goal)
    min_z = min(z for x, y, z in corners)
    # allow a bit below zero to compensate numerical inaccuracies
    if min_z < -1e-10:
        raise InvalidGoalError(
            "Position of a corner is too low (z = {}).".format(min_z),
            goal.position,
            goal.orientation,
        )


def evaluate_state(goal_pose, actual_pose, difficulty):
    """Compute cost of a given cube pose.  Less is better.

    Args:
        goal_pose:  Goal pose of the cube.
        actual_pose:  Actual pose of the cube.
        difficulty:  The difficulty level of the goal (see
            :func:`sample_goal`).  The metric for evaluating a state differs
            depending on the level.

    Returns:
        Cost of the actual pose w.r.t. to the goal pose.  Lower value means
        that the actual pose is closer to the goal.  Zero if actual == goal.
    """
    def weighted_position_error():
        range_xy_dist = _ARENA_RADIUS * 2
        range_z_dist = _max_height

        xy_dist = np.linalg.norm(
            goal_pose.position[:2] - actual_pose.position[:2]
        )
        z_dist = abs(goal_pose.position[2] - actual_pose.position[2])

        # weight xy- and z-parts by their expected range
        return (xy_dist / range_xy_dist + z_dist / range_z_dist) / 2

    if difficulty in (1, 2, 3):
        # consider only 3d position
        return weighted_position_error()
    elif difficulty == 4:
        # consider whole pose
        scaled_position_error = weighted_position_error()

        # https://stackoverflow.com/a/21905553
        goal_rot = Rotation.from_quat(goal_pose.orientation)
        actual_rot = Rotation.from_quat(actual_pose.orientation)
        error_rot = goal_rot.inv() * actual_rot
        orientation_error = error_rot.magnitude()

        # scale both position and orientation error to be within [0, 1] for
        # their expected ranges
        scaled_orientation_error = orientation_error / np.pi

        scaled_error = (scaled_position_error + scaled_orientation_error) / 2
        return scaled_error

        # Use DISP distance (max. displacement of the corners)
        # goal_corners = get_cube_corner_positions(goal_pose)
        # actual_corners = get_cube_corner_positions(actual_pose)
        # disp = max(np.linalg.norm(goal_corners - actual_corners, axis=1))
    else:
        raise ValueError("Invalid difficulty %d" % difficulty)


def goal_to_json(goal):
    """Convert goal object to JSON string.

    Args:
        goal (Pose):  A goal pose.

    Returns:
        str:  JSON string representing the goal pose.
    """
    goal_dict = {
        "position": goal.position.tolist(),
        "orientation": goal.orientation.tolist(),
    }
    return json.dumps(goal_dict)


def goal_from_json(json_string):
    """Create goal object from JSON string.

    Args:
        json_string (str):  JSON string containing "position" and "orientation"
            keys.

    Returns:
        Pose:  Goal pose.
    """
    goal_dict = json.loads(json_string)
    goal = Pose()
    goal.position = np.array(goal_dict["position"])
    goal.orientation = np.array(goal_dict["orientation"])
    return goal
