import pytest
import json

import numpy as np

from trifinger_simulation.tasks import move_cube_on_trajectory as mct


def test_validate_sampling():
    # Make sure that the sampled trajectories are actually valid
    for i in range(42):
        traj = mct.sample_trajectory()
        try:
            mct.validate_trajectory(traj)
        except Exception as e:
            pytest.fail(f"Unexpected error {e} for trajectory {traj}")


def test_get_active_goal():
    traj = [
        (0, (0, 0, 0)),
        (100, (1, 1, 1)),
        (200, (2, 2, 2)),
        (300, (3, 3, 3)),
    ]

    assert mct.get_active_goal(traj, 0) == (0, 0, 0)
    assert mct.get_active_goal(traj, 42) == (0, 0, 0)
    assert mct.get_active_goal(traj, 99) == (0, 0, 0)
    assert mct.get_active_goal(traj, 100) == (1, 1, 1)
    assert mct.get_active_goal(traj, 142) == (1, 1, 1)
    assert mct.get_active_goal(traj, 199) == (1, 1, 1)
    assert mct.get_active_goal(traj, 200) == (2, 2, 2)
    assert mct.get_active_goal(traj, 299) == (2, 2, 2)
    assert mct.get_active_goal(traj, 300) == (3, 3, 3)
    assert mct.get_active_goal(traj, 301) == (3, 3, 3)


def test_validate_trajectory():
    # Validate various (mostly ill-defined) trajectories and verify the proper
    # response.

    good = [
        (0, (0, 0, 0.05)),
        (100, (0.1, 0, 0.1)),
        (200, (0.02, 0.02, 0.05)),
        (300, (0.04, 0.01, 0.07)),
    ]
    try:
        mct.validate_trajectory(good)
    except Exception as e:
        pytest.fail(f"Unexpected error {e}")

    # does not start at 0
    bad = [
        (1, (0, 0, 0.05)),
        (100, (0.1, 0, 0.1)),
        (200, (0.02, 0.02, 0.05)),
        (300, (0.04, 0.01, 0.07)),
    ]
    with pytest.raises(ValueError):
        mct.validate_trajectory(bad)

    # two goals start at same step
    bad = [
        (0, (0, 0, 0.05)),
        (100, (0.1, 0, 0.1)),
        (100, (0.02, 0.02, 0.05)),
        (300, (0.04, 0.01, 0.07)),
    ]
    with pytest.raises(ValueError):
        mct.validate_trajectory(bad)

    # bad order of goal start times
    bad = [
        (0, (0, 0, 0.05)),
        (200, (0.02, 0.02, 0.05)),
        (100, (0.1, 0, 0.1)),
        (300, (0.04, 0.01, 0.07)),
    ]
    with pytest.raises(ValueError):
        mct.validate_trajectory(bad)

    # trajectory must not be empty
    bad = []
    with pytest.raises(ValueError):
        mct.validate_trajectory(bad)

    # ill-defined position on first goal
    bad = [
        (0, (0, 0)),
        (100, (0.1, 0, 0.1)),
    ]
    with pytest.raises(ValueError):
        mct.validate_trajectory(bad)

    # z=0 is out of range
    bad = [
        (0, (0, 0, 0)),
        (100, (0.1, 0, 0.1)),
    ]
    with pytest.raises(mct.move_cube.InvalidGoalError):
        mct.validate_trajectory(bad)

    # x=1 is out of range
    bad = [
        (0, (0, 0, 0.05)),
        (100, (1, 0, 0.1)),
    ]
    with pytest.raises(mct.move_cube.InvalidGoalError):
        mct.validate_trajectory(bad)


def test_evaluate_state():
    traj = [
        (0, (0, 0, 0.05)),
        (100, (0.1, 0, 0.1)),
        (200, (0.02, 0.02, 0.05)),
        (300, (0.04, 0.01, 0.07)),
    ]

    # using the current goal as actual position should always result in a cost
    # of 0
    for t, goal in traj:
        assert mct.evaluate_state(traj, t, goal) == 0

    # verify that using a different actual position results in a non-zero cost
    assert mct.evaluate_state(traj, 0, (0, 0, 0)) > 0

    # cost should differ for the different goals
    assert mct.evaluate_state(traj, 0, (0, 0, 0)) != mct.evaluate_state(
        traj, 110, (0, 0, 0)
    )


def test_json_basic():
    traj = [
        (0, (0, 0, 0.05)),
        (100, (0.1, 0, 0.1)),
        (200, (0.02, 0.02, 0.05)),
        (300, (0.04, 0.01, 0.07)),
    ]

    # verify that serializing and deserializing again results in the same
    # trajectory
    json_str = mct.trajectory_to_json(traj)
    traj2 = json.loads(json_str)

    for i in range(len(traj)):
        assert traj[i][0] == traj2[i][0]
        np.testing.assert_array_equal(traj[i][1], traj2[i][1])


def test_json_numpy():
    traj = [
        (0, np.array((0, 0, 0.05))),
        (100, np.array((0.1, 0, 0.1))),
        (200, np.array((0.02, 0.02, 0.05))),
        (300, np.array((0.04, 0.01, 0.07))),
    ]

    # verify that serializing and deserializing again results in the same
    # trajectory
    json_str = mct.trajectory_to_json(traj)
    traj2 = json.loads(json_str)

    for i in range(len(traj)):
        assert traj[i][0] == traj2[i][0]
        np.testing.assert_array_equal(traj[i][1], traj2[i][1])
