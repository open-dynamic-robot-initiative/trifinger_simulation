#!/usr/bin/env python3
import pytest
import numpy as np
from scipy.spatial.transform import Rotation

from trifinger_simulation.tasks import move_cube


def test_get_cube_corner_positions():
    # cube half width
    chw = move_cube._CUBE_WIDTH / 2
    # no transformation
    expected_origin_corners = np.array(
        [
            [-chw, -chw, -chw],
            [-chw, -chw, +chw],
            [-chw, +chw, -chw],
            [-chw, +chw, +chw],
            [+chw, -chw, -chw],
            [+chw, -chw, +chw],
            [+chw, +chw, -chw],
            [+chw, +chw, +chw],
        ]
    )
    origin = move_cube.Pose()
    origin_corners = move_cube.get_cube_corner_positions(origin)
    np.testing.assert_array_almost_equal(
        expected_origin_corners, origin_corners
    )

    # only translation
    expected_translated_corners = np.array(
        [
            [-chw + 1, -chw + 2, -chw + 3],
            [-chw + 1, -chw + 2, +chw + 3],
            [-chw + 1, +chw + 2, -chw + 3],
            [-chw + 1, +chw + 2, +chw + 3],
            [+chw + 1, -chw + 2, -chw + 3],
            [+chw + 1, -chw + 2, +chw + 3],
            [+chw + 1, +chw + 2, -chw + 3],
            [+chw + 1, +chw + 2, +chw + 3],
        ]
    )
    translated = move_cube.get_cube_corner_positions(
        move_cube.Pose([1, 2, 3], [0, 0, 0, 1])
    )
    np.testing.assert_array_almost_equal(
        expected_translated_corners, translated
    )

    # only rotation
    rot_z90 = Rotation.from_euler("z", 90, degrees=True).as_quat()
    expected_rotated_corners = np.array(
        [
            [+chw, -chw, -chw],
            [+chw, -chw, +chw],
            [-chw, -chw, -chw],
            [-chw, -chw, +chw],
            [+chw, +chw, -chw],
            [+chw, +chw, +chw],
            [-chw, +chw, -chw],
            [-chw, +chw, +chw],
        ]
    )
    rotated = move_cube.get_cube_corner_positions(
        move_cube.Pose([0, 0, 0], rot_z90)
    )
    np.testing.assert_array_almost_equal(expected_rotated_corners, rotated)

    # both rotation and translation
    expected_both_corners = np.array(
        [
            [+chw + 1, -chw + 2, -chw + 3],
            [+chw + 1, -chw + 2, +chw + 3],
            [-chw + 1, -chw + 2, -chw + 3],
            [-chw + 1, -chw + 2, +chw + 3],
            [+chw + 1, +chw + 2, -chw + 3],
            [+chw + 1, +chw + 2, +chw + 3],
            [-chw + 1, +chw + 2, -chw + 3],
            [-chw + 1, +chw + 2, +chw + 3],
        ]
    )
    both = move_cube.get_cube_corner_positions(
        move_cube.Pose([1, 2, 3], rot_z90)
    )
    np.testing.assert_array_almost_equal(expected_both_corners, both)


def test_sample_goal_difficulty_1_no_initial_pose():
    for i in range(1000):
        goal = move_cube.sample_goal(difficulty=1)
        # verify the goal is valid (i.e. within the allowed ranges)
        try:
            move_cube.validate_goal(goal)
        except move_cube.InvalidGoalError as e:
            pytest.fail(
                msg="Invalid goal: {}  pose is {}, {}".format(
                    e, e.position, e.orientation
                ),
            )

        # verify the goal satisfies conditions of difficulty 1
        # always on ground
        assert goal.position[2] == (move_cube._CUBE_WIDTH / 2)

        # no orientation
        np.testing.assert_array_equal(goal.orientation, [0, 0, 0, 1])


def test_sample_goal_difficulty_2_no_initial_pose():
    for i in range(1000):
        goal = move_cube.sample_goal(difficulty=2)
        # verify the goal is valid (i.e. within the allowed ranges)
        try:
            move_cube.validate_goal(goal)
        except move_cube.InvalidGoalError as e:
            pytest.fail(
                msg="Invalid goal: {}  pose is {}, {}".format(
                    e, e.position, e.orientation
                ),
            )

        # verify the goal satisfies conditions of difficulty 2
        assert goal.position[2] <= move_cube._max_height
        assert goal.position[2] >= move_cube._min_height

        # no orientation
        np.testing.assert_array_equal(goal.orientation, [0, 0, 0, 1])


def test_sample_goal_difficulty_3_no_initial_pose():
    for i in range(1000):
        goal = move_cube.sample_goal(difficulty=3)
        # verify the goal is valid (i.e. within the allowed ranges)
        try:
            move_cube.validate_goal(goal)
        except move_cube.InvalidGoalError as e:
            pytest.fail(
                msg="Invalid goal: {}  pose is {}, {}".format(
                    e, e.position, e.orientation
                ),
            )

        # verify the goal satisfies conditions of difficulty 2
        assert goal.position[2] <= move_cube._max_height
        assert goal.position[2] >= move_cube._min_height

        # no orientation
        np.testing.assert_array_equal(goal.orientation, [0, 0, 0, 1])


def test_sample_goal_difficulty_4_no_initial_pose():
    for i in range(1000):
        goal = move_cube.sample_goal(difficulty=4)
        # verify the goal is valid (i.e. within the allowed ranges)
        try:
            move_cube.validate_goal(goal)
        except move_cube.InvalidGoalError as e:
            pytest.fail(
                msg="Invalid goal: {}  pose is {}, {}".format(
                    e, e.position, e.orientation
                ),
            )

        # verify the goal satisfies conditions of difficulty 2
        assert goal.position[2] <= move_cube._max_height
        assert goal.position[2] >= move_cube._min_height


def test_evaluate_state_difficulty_1():
    difficulty = 1
    pose_origin = move_cube.Pose()
    pose_trans = move_cube.Pose(position=[1, 2, 3])
    pose_rot = move_cube.Pose(
        orientation=Rotation.from_euler("z", 0.42).as_quat()
    )
    pose_both = move_cube.Pose(
        [1, 2, 3], Rotation.from_euler("z", 0.42).as_quat()
    )

    # needs to be zero for exact match
    cost = move_cube.evaluate_state(pose_origin, pose_origin, difficulty)
    assert cost == 0

    # None-zero if there is translation, rotation is ignored
    assert move_cube.evaluate_state(pose_origin, pose_trans, difficulty) != 0
    assert move_cube.evaluate_state(pose_origin, pose_rot, difficulty) == 0
    assert move_cube.evaluate_state(pose_origin, pose_both, difficulty) != 0


def test_evaluate_state_difficulty_2():
    difficulty = 2
    pose_origin = move_cube.Pose()
    pose_trans = move_cube.Pose(position=[1, 2, 3])
    pose_rot = move_cube.Pose(
        orientation=Rotation.from_euler("z", 0.42).as_quat()
    )
    pose_both = move_cube.Pose(
        [1, 2, 3], Rotation.from_euler("z", 0.42).as_quat()
    )

    # needs to be zero for exact match
    cost = move_cube.evaluate_state(pose_origin, pose_origin, difficulty)
    assert cost == 0

    # None-zero if there is translation, rotation is ignored
    assert move_cube.evaluate_state(pose_origin, pose_trans, difficulty) != 0
    assert move_cube.evaluate_state(pose_origin, pose_rot, difficulty) == 0
    assert move_cube.evaluate_state(pose_origin, pose_both, difficulty) != 0


def test_evaluate_state_difficulty_3():
    difficulty = 3
    pose_origin = move_cube.Pose()
    pose_trans = move_cube.Pose(position=[1, 2, 3])
    pose_rot = move_cube.Pose(
        orientation=Rotation.from_euler("z", 0.42).as_quat()
    )
    pose_both = move_cube.Pose(
        [1, 2, 3], Rotation.from_euler("z", 0.42).as_quat()
    )

    # needs to be zero for exact match
    cost = move_cube.evaluate_state(pose_origin, pose_origin, difficulty)
    assert cost == 0

    # None-zero if there is translation, rotation is ignored
    assert move_cube.evaluate_state(pose_origin, pose_trans, difficulty) != 0
    assert move_cube.evaluate_state(pose_origin, pose_rot, difficulty) == 0
    assert move_cube.evaluate_state(pose_origin, pose_both, difficulty) != 0


def test_evaluate_state_difficulty_4():
    difficulty = 4
    pose_origin = move_cube.Pose()
    pose_trans = move_cube.Pose(position=[1, 2, 3])
    pose_rot = move_cube.Pose(
        orientation=Rotation.from_euler("z", 0.42).as_quat()
    )
    pose_both = move_cube.Pose(
        [1, 2, 3], Rotation.from_euler("z", 0.42).as_quat()
    )

    # needs to be zero for exact match
    cost = move_cube.evaluate_state(pose_origin, pose_origin, difficulty)
    assert cost == 0

    # None-zero if there is translation, rotation or both
    assert move_cube.evaluate_state(pose_origin, pose_trans, difficulty) != 0
    assert move_cube.evaluate_state(pose_origin, pose_rot, difficulty) != 0
    assert move_cube.evaluate_state(pose_origin, pose_both, difficulty) != 0


def test_evaluate_state_dict():
    """Test evaluate state using a dict instead of Pose."""
    difficulty = 4
    pose_origin = move_cube.Pose()
    dict_origin = {"position": [0, 0, 0], "orientation": [0, 0, 0, 1]}
    pose_pose = move_cube.Pose(
        [1, 2, 3], Rotation.from_euler("z", 0.42).as_quat()
    )
    dict_pose = {
        "position": [1, 2, 3],
        "orientation": Rotation.from_euler("z", 0.42).as_quat(),
    }

    # needs to be zero for exact match
    pose_cost = move_cube.evaluate_state(pose_origin, pose_pose, difficulty)
    dict_cost = move_cube.evaluate_state(dict_origin, dict_pose, difficulty)
    assert pose_cost == dict_cost


def test_validate_goal():
    half_width = move_cube._CUBE_WIDTH / 2
    yaw_rotation = Rotation.from_euler("z", 0.42).as_quat()
    full_rotation = Rotation.from_euler("zxz", [0.42, 0.1, -2.3]).as_quat()

    # test some valid goals
    try:
        move_cube.validate_goal(
            move_cube.Pose([0, 0, half_width], [0, 0, 0, 1])
        )
    except Exception as e:
        pytest.fail("Valid goal was considered invalid because %s" % e)

    try:
        move_cube.validate_goal(
            move_cube.Pose([0.05, -0.1, half_width], yaw_rotation)
        )
    except Exception as e:
        pytest.fail("Valid goal was considered invalid because %s" % e)

    try:
        move_cube.validate_goal(
            move_cube.Pose([-0.12, 0.0, 0.06], full_rotation)
        )
    except Exception as e:
        pytest.fail("Valid goal was considered invalid because %s" % e)

    # test some invalid goals

    # invalid values
    with pytest.raises(ValueError):
        move_cube.validate_goal(move_cube.Pose([0, 0], [0, 0, 0, 1]))
    with pytest.raises(ValueError):
        move_cube.validate_goal(move_cube.Pose([0, 0, 0], [0, 0, 1]))

    # invalid positions
    with pytest.raises(move_cube.InvalidGoalError):
        move_cube.validate_goal(
            move_cube.Pose([0.3, 0, half_width], [0, 0, 0, 1])
        )
    with pytest.raises(move_cube.InvalidGoalError):
        move_cube.validate_goal(
            move_cube.Pose([0, -0.3, half_width], [0, 0, 0, 1])
        )
    with pytest.raises(move_cube.InvalidGoalError):
        move_cube.validate_goal(move_cube.Pose([0, 0, 0.3], [0, 0, 0, 1]))
    with pytest.raises(move_cube.InvalidGoalError):
        move_cube.validate_goal(move_cube.Pose([0, 0, 0], [0, 0, 0, 1]))
    with pytest.raises(move_cube.InvalidGoalError):
        move_cube.validate_goal(move_cube.Pose([0, 0, -0.01], [0, 0, 0, 1]))

    # valid CoM position but rotation makes it reach out of valid range
    with pytest.raises(move_cube.InvalidGoalError):
        move_cube.validate_goal(
            move_cube.Pose([0, 0, half_width], full_rotation)
        )


def test_validate_goal_dict():
    """Test validation of a goal that is passed as dict."""
    half_width = move_cube._CUBE_WIDTH / 2
    full_rotation = Rotation.from_euler("zxz", [0.42, 0.1, -2.3]).as_quat()

    # a valid goal
    try:
        move_cube.validate_goal(
            {"position": [-0.12, 0.0, 0.06], "orientation": full_rotation}
        )
    except Exception as e:
        pytest.fail("Valid goal was considered invalid because %s" % e)

    # some invalid goals

    # invalid values
    with pytest.raises(ValueError):
        move_cube.validate_goal(
            {"position": [0, 0], "orientation": [0, 0, 0, 1]}
        )

    # invalid positions
    with pytest.raises(move_cube.InvalidGoalError):
        move_cube.validate_goal(
            {"position": [0.3, 0, half_width], "orientation": [0, 0, 0, 1]}
        )

    # invalid dict keys
    with pytest.raises(KeyError):
        move_cube.validate_goal(
            {"wrong_key": [-0.12, 0.0, 0.06], "orientation": full_rotation}
        )
