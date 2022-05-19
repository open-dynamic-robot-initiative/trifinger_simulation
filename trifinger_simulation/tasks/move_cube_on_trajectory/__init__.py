"""Task: Move Cube on Trajectory

The goal of this task is to grasp a cube and move it from one goal to the next
on a given trajectory.

The trajectory is given as a list of tuples ``(t, goal_position)`` where ``t``
specifies the time step from which the goal position is active.  So the full
type is ``Sequence[Tuple[int, Sequence[float]]]``

Example:

.. code-block:: Python

    trajectory = [
        (0, (0, 0, 0.0325)),
        (4000, (0, 0.05, 0.0325)),
        (8000, (0.05, 0.05, 0.0325)),
    ]

The first goal ``(0, 0, 0.0325)`` is active at the beginning.  Starting from
time step 4000 the active goal switches to ``(0, 0.05, 0.0325)``.  At step 8000
``(0.05, 0.05, 0.0325)`` becomes the active goal and stays active until the end
of the run.

The duration of a run is 120000 steps (~2 minutes).  This value is also given
by :data:`EPISODE_LENGTH`.

The cost of each step is computed using :func:`evaluate_state`.
"""
import json
import typing

import numpy as np

from trifinger_simulation.tasks import move_cube

# define some types for type hints
Position = typing.Sequence[float]
TrajectoryStep = typing.Tuple[int, Position]
Trajectory = typing.Sequence[TrajectoryStep]


#: Duration of the episode in time steps (corresponds to ~2 minutes).
EPISODE_LENGTH = 2 * 60 * 1000
#: Number of time steps for which the first goal in the trajectory is active.
FIRST_GOAL_DURATION = 30 * 1000
#: Number of time steps for which following goals in the trajectory are active.
GOAL_DURATION = 10 * 1000

#: Goal difficulty that is used for sampling steps of the trajectory
GOAL_DIFFICULTY = 3

#: The initial position of the cube at the beginning of an episode.
INITIAL_CUBE_POSITION = (0, 0, move_cube._CUBE_WIDTH / 2)


def get_active_goal(
    trajectory: Trajectory, time_index: int
) -> typing.Optional[Position]:
    """Get the trajectory goal that is active at the given time step.

    Args:
        trajectory: The trajectory.
        time_index: Index of the desired time step.

    Returns:
        The goal from the given trajectory that is active at the given time
        step.
    """
    previous_goal = None
    for t, goal in trajectory:
        if time_index < t:
            break
        previous_goal = goal

    return previous_goal


def seed(seed: int):
    """Set random seed for this module."""
    # all sampling is actually happening in the move_cube module
    move_cube.seed(seed)


def sample_goal() -> Trajectory:
    """Sample a goal trajectory with random steps.

    The number of goals in the trajectory is depending on the episode length
    (see :data:`EPISODE_LENGTH`).  The first goal has a duration of
    :data:`FIRST_GOAL_DURATION` steps, the following ones a duration of
    :data:`GOAL_DURATION`.

    Returns:
        Trajectory as a list of tuples ``(t, position)`` where ``t`` marks the
        time step from which the goal is active.
    """
    trajectory = []

    first_goal = move_cube.sample_goal(GOAL_DIFFICULTY)
    trajectory.append((0, first_goal.position))
    t = FIRST_GOAL_DURATION

    while t < EPISODE_LENGTH:
        goal = move_cube.sample_goal(GOAL_DIFFICULTY)
        trajectory.append((t, goal.position))
        t += GOAL_DURATION

    return trajectory


def validate_goal(trajectory: Trajectory):
    """Checks if the given trajectory is valid, raises error if not.

    Raises:
        ValueError:  If there are any structural issues.
        move_cube.InvalidGoalError:  If a position exceeds the allowed goal
            space.
    """
    previous_t = -1

    if not trajectory:
        raise ValueError("Trajectory is empty")

    if trajectory[0][0] != 0:
        raise ValueError("First goal does not start at t=0")

    for i, (t, goal) in enumerate(trajectory):
        if t <= previous_t:
            raise ValueError(f"Goal {i} starts before previous goal")
        previous_t = t

        move_cube.validate_goal(move_cube.Pose(position=goal))


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

        goal_json = trajectory_to_json(goal)

    except Exception as e:
        raise RuntimeError(
            "Failed to load goal configuration.  Make sure you provide a valid"
            " 'goal.json' in your code repository.\n"
            " Error: %s" % e
        )

    return goal_json


def evaluate_state(
    trajectory: Trajectory, time_index: int, actual_position: Position
):
    """Compute cost of a given cube pose at a given time step.  Less is better.

    The cost is computed using :func:`move_cube.evaluate_state` (difficulty=3)
    for the goal position that is active at the given time_index.

    Args:
        trajectory:  The trajectory based on which the cost is computed.
        time_index:  Index of the time step that is evaluated.
        actual_position:  Cube position at the specified time step.

    Returns:
        Cost of the actual position w.r.t. to the goal position of the active
        step in the trajectory.  Lower value means that the actual pose is
        closer to the goal.  Zero if actual == goal.
    """
    active_goal = get_active_goal(trajectory, time_index)

    # wrap positions in Pose objects
    actual_pose = move_cube.Pose(position=actual_position)
    goal_pose = move_cube.Pose(position=active_goal)

    return move_cube.evaluate_state(goal_pose, actual_pose, GOAL_DIFFICULTY)


class NumpyEncoder(json.JSONEncoder):
    """JSON encoder that handles NumPy arrays like lists.

    Taken from https://stackoverflow.com/a/47626762
    """

    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)


def trajectory_to_json(trajectory: Trajectory) -> str:
    """Serialize a trajectory into a JSON string."""
    # numpy arrays need to be converted to normal tuples
    return json.dumps(trajectory, cls=NumpyEncoder)
