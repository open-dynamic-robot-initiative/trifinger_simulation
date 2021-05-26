"""Task: Rearrange Dice

The goal of this task is to arrange multiple dice into a given pattern.

The pattern is given as a list of N target positions where N is the number of
dice:

.. code-block:: Python

    goal = [
        (0.10, 0.04, 0.01),
        (0.04, -0.08, 0.01),
        (0.0, 0.15, 0.01),
        ...
    ]

Since the single dice are indistinguishable the target positions are not linked
to a specific die, there should just be one die at each position in the end.

The duration of a run is 120000 steps (~2 minutes).  This value is also given
by :data:`EPISODE_LENGTH`.

The cost of each step is computed using the camera images.  Based on the
colour, it is determined how many "die pixels" are outside of the target
regions (see :func:`evaluate_state`).
"""
import itertools
import json
import random
import typing

import numpy as np
import cv2
from scipy.spatial.transform import Rotation

from trifinger_simulation import camera


#: Duration of the episode in time steps (corresponds to ~2 minutes).
EPISODE_LENGTH = 2 * 60 * 1000

#: Radius of the arena in which target positions are sampled [m].
ARENA_RADIUS = 0.19

#: Number of dice in the arena
NUM_DICE = 25

#: Width of a die [m].
DIE_WIDTH = 0.022

#: Tolerance that is added to the target box width [m].
TOLERANCE = 0.003

#: Width of the target box in which the die has to be placed [m].
TARGET_WIDTH = DIE_WIDTH + TOLERANCE

#: Number of cells per row (one cell fits one die)
N_CELLS_PER_ROW = int(2 * ARENA_RADIUS / DIE_WIDTH)


# Helper types for type hints
Cell = typing.Tuple[int, int]
Position = typing.Sequence[float]
Goal = typing.Sequence[Position]


# random number generator used in this module
_rng = random.Random()


class InvalidGoalError(Exception):
    pass


class OutOfArenaError(InvalidGoalError):
    """Exception used to indicate that a goal position is outside the arena."""

    def __init__(self, position):
        super().__init__(f"Position {position} is outside the arena.")
        self.position = position


class NumpyEncoder(json.JSONEncoder):
    """JSON encoder that handles NumPy arrays like lists.

    Taken from https://stackoverflow.com/a/47626762
    """

    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)


def _cell_center_position(cell: Cell) -> Position:
    """Get 3d position of the cell centre."""
    n_half = N_CELLS_PER_ROW / 2
    px = (cell[0] - n_half) * DIE_WIDTH + DIE_WIDTH / 2
    py = (cell[1] - n_half) * DIE_WIDTH + DIE_WIDTH / 2
    pz = DIE_WIDTH / 2

    return (px, py, pz)


def _get_cell_corners_2d(
    pos: Position,
) -> typing.Tuple[Position, ...]:
    """Get 2d positions of the corners of the cell at the given position."""
    d = DIE_WIDTH / 2
    nppos = np.asarray(pos)[:2]

    return (
        nppos + (d, d),
        nppos + (d, -d),
        nppos + (-d, -d),
        nppos + (-d, d),
    )


def _get_cell_corners_3d(
    pos: Position,
) -> np.ndarray:
    """Get 3d positions of the corners of the cell at the given position."""
    d = DIE_WIDTH / 2
    nppos = np.asarray(pos)

    # order of the corners is the same as in the cube model of the
    # trifinger_object_tracking package
    # people.tue.mpg.de/mpi-is-software/robotfingers/docs/trifinger_object_tracking/doc/cube_model.html
    return np.array(
        (
            nppos + (d, -d, d),
            nppos + (d, d, d),
            nppos + (-d, d, d),
            nppos + (-d, -d, d),
            nppos + (d, -d, -d),
            nppos + (d, d, -d),
            nppos + (-d, d, -d),
            nppos + (-d, -d, -d),
        )
    )


FACE_CORNERS = (
    (0, 1, 2, 3),
    (4, 5, 1, 0),
    (5, 6, 2, 1),
    (7, 6, 2, 3),
    (4, 7, 3, 0),
    (4, 5, 6, 7),
)


def _is_cell_position_inside_arena(pos: Position) -> bool:
    """Check if cell is inside the arena circle."""
    corners = _get_cell_corners_2d(pos)

    corner_dists_to_center = np.array([np.linalg.norm(c) for c in corners])
    return np.all(corner_dists_to_center <= ARENA_RADIUS)


def _is_cell_inside_arena(cell: Cell) -> bool:
    """Check if cell is inside the arena circle."""
    pos = _cell_center_position(cell)
    return _is_cell_position_inside_arena(pos)


def _get_grid_cells() -> typing.List[Cell]:
    """Get list of all grid cells that are completely inside the arena."""
    # start with a rectangular grid
    cells = itertools.product(range(N_CELLS_PER_ROW), range(N_CELLS_PER_ROW))

    # filter out cells that are not inside the arena circle
    inside_arena_cells = [c for c in cells if _is_cell_inside_arena(c)]

    return inside_arena_cells


def goal_to_json(goal: Goal) -> str:
    """Convert goal to JSON string."""
    return json.dumps(goal, cls=NumpyEncoder)


def seed(seed: int):
    """Set random seed for this module."""
    global _rng
    _rng = random.Random(seed)


def sample_goal():
    """Sample a random list of die goal positions."""
    cells = _get_grid_cells()
    target_cells = _rng.sample(cells, NUM_DICE)
    target_positions = [_cell_center_position(c) for c in target_cells]

    return target_positions


def validate_goal(goal):
    """Verify that the goal has the proper shape and all positions are valid.

    Raises:
        OutOfArenaError:  If a die position is outside the valid range.
        InvalidGoalError:  If the goal does not have the expected shape.
    """
    if len(goal) != NUM_DICE:
        raise InvalidGoalError(
            "Wrong number of positions.  Expected {}, got {}".format(
                NUM_DICE, len(goal)
            )
        )

    for i, pos in enumerate(goal):
        if len(pos) != 3:
            raise InvalidGoalError(f"Position {i} has invalid shape.")

        if not _is_cell_position_inside_arena(pos):
            raise OutOfArenaError(pos)

        if pos[2] < DIE_WIDTH / 2:
            raise OutOfArenaError(pos)


def json_goal_from_config(filename: str) -> str:
    """Load or sample a goal based on the given goal config file.

    Args:
        filename: Path to the goal config JSON file.  If it contains an entry
            "goal", its value is used as goal.  Otherwise a random goal is
            sampled.

    Returns:
        The goal as JSON-encoded string.
    """
    try:
        with open(filename, "r") as f:
            goalconfig = json.load(f)

        if "goal" in goalconfig:
            goal = goalconfig["goal"]
            validate_goal(goal)
        else:
            goal = sample_goal()

        goal_json = json.dumps(goal, cls=NumpyEncoder)

    except Exception as e:
        raise RuntimeError(
            "Failed to load goal configuration.  Make sure you provide a valid"
            " 'goal.json' in your code repository.\n"
            " Error: %s" % e
        )

    return goal_json


def evaluate_state(
    goal_masks: typing.Sequence[np.ndarray],
    actual_masks: typing.Sequence[np.ndarray],
) -> float:
    """Compute cost of a given state.  Less is better.

    The cost is computed as the number of "die pixels" in the actual masks that
    do not overlap with the goal mask::

        cost = count(actual_masks AND (NOT goal_masks))

    Args:
        goal_masks: Masks of the desired die positions in the camera images,
            one mask per camera.  Use :func:`generate_goal_mask` to generate
            the goal mask for a given goal.
        actual_masks: Masks of the actual die positions in the camera images,
            one mask per camera using the same order as ``goal_masks``.

    Returns:
        The cost of the given state.
    """
    ...
    # compute the actual die pixels outside of the goal mask
    outside_goal = np.logical_and(actual_masks, np.logical_not(goal_masks))
    num_outside_pixels = np.count_nonzero(outside_goal)

    return num_outside_pixels


def visualize_2d(target_positions: Goal):
    """Visualise the target positions in 2d.

    Shows a top-down view of the arena with the goal positions marked by
    squares.

    Args:
        target_positions: The goal that is visualised.
    """
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots()

    ax.set_xlim([-ARENA_RADIUS, ARENA_RADIUS])
    ax.set_ylim([-ARENA_RADIUS, ARENA_RADIUS])
    ax.set_aspect("equal", "box")

    for p in target_positions:
        ax.add_artist(
            plt.Rectangle(
                xy=(p[0] - DIE_WIDTH / 2.0, p[1] - DIE_WIDTH / 2.0),
                color="g",
                width=TARGET_WIDTH,
                height=TARGET_WIDTH,
            )
        )

    circle = plt.Circle((0, 0), ARENA_RADIUS, color="black", fill=False)
    ax.add_patch(circle)

    plt.show()


def generate_goal_mask(
    camera_parameters: typing.Sequence[camera.CameraParameters], goal: Goal
) -> typing.List[np.ndarray]:
    """Generate goal masks that can be used with :func:`evaluate_state`.

    A goal mask is a single-channel image where the areas at which dice are
    supposed to be placed are white and everything else is black.  So it
    corresponds more or less to a segmentation mask where all dice are at the
    goal positions.

    For rendering the mask, :data:`TARGET_WIDTH` is used for the die width to
    add some tolerance.

    Args:
        camera_parameters: List of camera parameters, one per camera.
        goal: The goal die positions.

    Returns:
        List of masks.  The number and order of masks corresponds to the input
        ``camera_parameters``.
    """
    masks = []
    for cam in camera_parameters:
        mask = np.zeros((cam.height, cam.width), dtype=np.uint8)

        # get camera position and orientation separately
        tvec = cam.tf_world_to_camera[:3, 3]
        rmat = cam.tf_world_to_camera[:3, :3]
        rvec = Rotation.from_matrix(rmat).as_rotvec()

        for pos in goal:
            corners = _get_cell_corners_3d(pos)

            # project corner points into the image
            projected_corners, _ = cv2.projectPoints(
                corners,
                rvec,
                tvec,
                cam.camera_matrix,
                cam.distortion_coefficients,
            )

            # draw faces in mask
            for face_corner_idx in FACE_CORNERS:
                points = np.array(
                    [projected_corners[i] for i in face_corner_idx],
                    dtype=np.int32,
                )
                mask = cv2.fillConvexPoly(mask, points, 255)

        masks.append(mask)

    return masks
