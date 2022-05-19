import pytest
import json
import pathlib

import numpy as np

from trifinger_simulation.tasks import rearrange_dice as task


@pytest.fixture
def test_data_dir():
    """Return path to the data directory of this test file."""
    p = pathlib.Path(__file__)
    p = p.parent / p.stem  # same path but without file extension
    assert p.is_dir(), f"{p} is not a directory."
    return p


def test_validate_sampling():
    # Make sure that the sampled goalectories are actually valid
    for i in range(42):
        goal = task.sample_goal()
        try:
            task.validate_goal(goal)
        except Exception as e:
            pytest.fail(f"Unexpected error {e} for goal {goal}")


def test_validate_goal():
    height = task.DIE_WIDTH / 2
    good = [
        (0.10, 0.04, height),
        (0.04, -0.08, height),
        (0.0, 0.15, height),
        (0.0, 0.0, height),
        (-0.1, -0.02, height),
    ] * 5  # assumes task.NUM_DICE == 25

    try:
        task.validate_goal(good)
    except Exception as e:
        pytest.fail(f"Unexpected error {e}")

    # height = 0
    bad = [
        (0.10, 0.04, 0),
    ] * task.NUM_DICE
    with pytest.raises(task.OutOfArenaError):
        task.validate_goal(bad)

    # x too low
    bad = [
        (-0.2, 0.0, height),
    ] * task.NUM_DICE
    with pytest.raises(task.OutOfArenaError):
        task.validate_goal(bad)

    # x too large
    bad = [
        (+0.2, 0.0, height),
    ] * task.NUM_DICE
    with pytest.raises(task.OutOfArenaError):
        task.validate_goal(bad)

    # y too low
    bad = [
        (0.0, -0.2, height),
    ] * task.NUM_DICE
    with pytest.raises(task.OutOfArenaError):
        task.validate_goal(bad)

    # y too large
    bad = [
        (0.0, +0.2, height),
    ] * task.NUM_DICE
    with pytest.raises(task.OutOfArenaError):
        task.validate_goal(bad)

    # wrong number of positions
    bad = [
        (0.10, 0.04, height),
        (0.04, -0.08, height),
        (0.0, 0.15, height),
        (0.0, 0.0, height),
        (-0.1, -0.02, height),
    ]
    with pytest.raises(task.InvalidGoalError):
        task.validate_goal(bad)

    # bad format
    bad = [
        (0.04, -0.08),
    ] * task.NUM_DICE
    with pytest.raises(task.InvalidGoalError):
        task.validate_goal(bad)

    # empty
    bad = []
    with pytest.raises(task.InvalidGoalError):
        task.validate_goal(bad)


def test_json_goal_from_config_good(test_data_dir):
    expected_goal = [
        [0.132, 0.065, 0.011],
        [0.0, -0.088, 0.011],
        [0.022, 0.0, 0.011],
        [0.065, 0.132, 0.011],
        [-0.066, 0.154, 0.011],
        [-0.11, -0.11, 0.011],
        [0.109, -0.131, 0.011],
        [-0.11, -0.044, 0.011],
        [0.022, -0.066, 0.011],
        [0.044, -0.131, 0.011],
        [-0.066, 0.132, 0.011],
        [0.022, -0.131, 0.011],
        [0.065, 0.088, 0.011],
        [0.176, -0.0222, 0.011],
        [0.065, 0.0, 0.011],
        [-0.131, -0.0220, 0.011],
        [-0.153, 0.044, 0.011],
        [-0.044, 0.022, 0.011],
        [0.044, 0.0, 0.011],
        [0.109, 0.0, 0.011],
        [-0.153, -0.0222, 0.011],
        [-0.11, 0.044, 0.011],
        [0.132, -0.11, 0.011],
        [0.065, 0.109, 0.011],
        [-0.066, 0.044, 0.011],
    ]

    actual_goal_json = task.json_goal_from_config(
        test_data_dir / "good_goal.json"
    )
    actual_goal = json.loads(actual_goal_json)

    assert expected_goal == actual_goal


def test_json_goal_from_config_no_goal(test_data_dir):
    try:
        # no_goal.json does not define a goal.  This should not lead to a
        # failure, though but json_goal_from_config should return a randomly
        # sampled one
        goal_json = task.json_goal_from_config(test_data_dir / "no_goal.json")
        goal = json.loads(goal_json)
        task.validate_goal(goal)
    except Exception:
        pytest.fail()


def test_json_goal_from_config_bad(test_data_dir):
    with pytest.raises(RuntimeError):
        task.json_goal_from_config(test_data_dir / "bad_goal.json")


def test_json_goal_from_config_no_file():
    with pytest.raises(RuntimeError):
        task.json_goal_from_config("/this/file/does/not/exist.json")


def test_evaluate_state_value_1():
    goal = [
        np.array(
            [
                [0, 0, 0],
                [0, 1, 1],
                [0, 1, 1],
            ]
        ),
        np.array(
            [
                [1, 1, 0],
                [1, 1, 0],
                [0, 0, 0],
            ]
        ),
        np.array(
            [
                [0, 1, 0],
                [1, 0, 1],
                [0, 1, 0],
            ]
        ),
    ]

    actual = [
        np.array(
            [
                [0, 0, 0],
                [1, 1, 0],
                [1, 1, 0],
            ]
        ),
        np.array(
            [
                [1, 1, 0],
                [1, 1, 0],
                [0, 0, 0],
            ]
        ),
        np.array(
            [
                [0, 0, 1],
                [0, 0, 1],
                [0, 0, 1],
            ]
        ),
    ]

    cost = task.evaluate_state(goal, actual)
    # cost is number of actual pixels outside of goal
    assert cost == 4


def test_evaluate_state_value_255():
    goal = [
        np.array(
            [
                [0, 0, 0],
                [0, 255, 255],
                [0, 255, 255],
            ]
        ),
        np.array(
            [
                [255, 255, 0],
                [255, 255, 0],
                [0, 0, 0],
            ]
        ),
        np.array(
            [
                [0, 255, 0],
                [255, 0, 255],
                [0, 255, 0],
            ]
        ),
    ]

    actual = [
        np.array(
            [
                [0, 0, 0],
                [255, 255, 0],
                [255, 255, 0],
            ]
        ),
        np.array(
            [
                [255, 255, 0],
                [255, 255, 0],
                [0, 0, 0],
            ]
        ),
        np.array(
            [
                [0, 0, 255],
                [0, 0, 255],
                [0, 0, 255],
            ]
        ),
    ]

    cost = task.evaluate_state(goal, actual)
    # cost is number of actual pixels outside of goal
    assert cost == 4
