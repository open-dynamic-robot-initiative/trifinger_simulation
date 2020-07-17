"""Functions for sampling, validating and evaluating "move cube" goals."""
import json
import random

import numpy as np
from scipy.spatial.transform import Rotation


# Number of time steps in one episode
episode_length = 5000  # TODO set actual value


_CUBE_WIDTH = 0.065
_ARENA_RADIUS = 0.195

_cube_3d_radius = _CUBE_WIDTH * np.sqrt(3) / 2
_max_cube_com_distance_to_center = _ARENA_RADIUS - _cube_3d_radius

_min_height = _CUBE_WIDTH / 2
_max_height = 0.1  # TODO


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
        #: Position (x, y, z).
        self.position = position
        #: Orientation as quaternion (x, y, z, w)
        self.orientation = orientation


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


def sample_goal(difficulty, current_pose=None):
    """Sample a goal pose for the cube.

    Args:
        difficulty (int):  Difficulty level.  The higher, the more difficult is
            the goal.  Possible levels are:

            - 1: Goal position on the table, no orientation.
            - 2: Lifting to a certain height.  No x,y-position, no orientation.
            - 3: Goal position in the air, no orientation.
            - 4: Goal pose in the air, including orientation.

        current_pose (Pose):  Current pose of the cube.  If set, it is
            considered while sampling the goal orientation.  If not set, it is
            assumed that the cube is currently at the origin and aligned with
            the axes of the world frame.  This is relevant for some difficulty
            levels to ensure that the goal orientation only differs from the
            current one by what is specified for that difficulty level (e.g.
            only rotation around z-axes for level 1).

    Returns:
        (Pose): Goal pose of the cube relative to the world frame.
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

        # make goal orientation relative to current orientation if given
        orientation = Rotation.from_euler("z", yaw)
        if current_pose is not None:
            orientation = orientation * Rotation.from_quat(
                current_pose.orientation
            )

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
        # FIXME set x,y to zero or to current position of cube?
        x = 0.0
        y = 0.0
        # FIXME use random or fixed height?
        z = 0.05
        orientation = np.array([0, 0, 0, 1])

    elif difficulty == 3:
        x, y = random_xy()
        z = random.uniform(_min_height, _max_height)
        orientation = np.array([0, 0, 0, 1])

    elif difficulty == 4:
        x, y = random_xy()
        z = random.uniform(_min_height, _max_height)
        # FIXME: only yaw rotation or completely arbitrary?
        orientation = random_yaw_orientation()

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
    if difficulty in (1, 2, 3):
        # consider only 3d position
        return np.linalg.norm(goal_pose.position - actual_pose.position)
    elif difficulty == 4:
        # consider whole pose
        # Use DISP distance (max. displacement of the corners)
        goal_corners = get_cube_corner_positions(goal_pose)
        actual_corners = get_cube_corner_positions(actual_pose)

        disp = max(np.linalg.norm(goal_corners - actual_corners, axis=1))
        return disp
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
