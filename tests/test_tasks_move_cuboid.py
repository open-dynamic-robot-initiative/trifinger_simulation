#!/usr/bin/env python3
import unittest
import numpy as np
from scipy.spatial.transform import Rotation

from trifinger_simulation.tasks import move_cuboid


class TestMoveCube(unittest.TestCase):
    """Test the functions of the "move cube" task module."""

    def test_get_cube_corner_positions(self):
        # cube half width
        chw = move_cuboid._CUBOID_HALF_SIZE
        # no transformation
        expected_origin_corners = np.array(
            [
                [-chw[0], -chw[1], -chw[2]],
                [-chw[0], -chw[1], +chw[2]],
                [-chw[0], +chw[1], -chw[2]],
                [-chw[0], +chw[1], +chw[2]],
                [+chw[0], -chw[1], -chw[2]],
                [+chw[0], -chw[1], +chw[2]],
                [+chw[0], +chw[1], -chw[2]],
                [+chw[0], +chw[1], +chw[2]],
            ]
        )
        origin = move_cuboid.Pose()
        origin_corners = move_cuboid.get_cube_corner_positions(origin)
        np.testing.assert_array_almost_equal(
            expected_origin_corners, origin_corners
        )

        # only translation
        expected_translated_corners = np.array(
            [
                [-chw[0] + 1, -chw[1] + 2, -chw[2] + 3],
                [-chw[0] + 1, -chw[1] + 2, +chw[2] + 3],
                [-chw[0] + 1, +chw[1] + 2, -chw[2] + 3],
                [-chw[0] + 1, +chw[1] + 2, +chw[2] + 3],
                [+chw[0] + 1, -chw[1] + 2, -chw[2] + 3],
                [+chw[0] + 1, -chw[1] + 2, +chw[2] + 3],
                [+chw[0] + 1, +chw[1] + 2, -chw[2] + 3],
                [+chw[0] + 1, +chw[1] + 2, +chw[2] + 3],
            ]
        )
        translated = move_cuboid.get_cube_corner_positions(
            move_cuboid.Pose([1, 2, 3], [0, 0, 0, 1])
        )
        np.testing.assert_array_almost_equal(
            expected_translated_corners, translated
        )

        # only rotation
        rot_z90 = Rotation.from_euler("z", 90, degrees=True).as_quat()
        expected_rotated_corners = np.array(
            [
                [+chw[1], -chw[0], -chw[2]],
                [+chw[1], -chw[0], +chw[2]],
                [-chw[1], -chw[0], -chw[2]],
                [-chw[1], -chw[0], +chw[2]],
                [+chw[1], +chw[0], -chw[2]],
                [+chw[1], +chw[0], +chw[2]],
                [-chw[1], +chw[0], -chw[2]],
                [-chw[1], +chw[0], +chw[2]],
            ]
        )
        rotated = move_cuboid.get_cube_corner_positions(
            move_cuboid.Pose([0, 0, 0], rot_z90)
        )
        np.testing.assert_array_almost_equal(expected_rotated_corners, rotated)

        # both rotation and translation
        expected_both_corners = np.array(
            [
                [+chw[1] + 1, -chw[0] + 2, -chw[2] + 3],
                [+chw[1] + 1, -chw[0] + 2, +chw[2] + 3],
                [-chw[1] + 1, -chw[0] + 2, -chw[2] + 3],
                [-chw[1] + 1, -chw[0] + 2, +chw[2] + 3],
                [+chw[1] + 1, +chw[0] + 2, -chw[2] + 3],
                [+chw[1] + 1, +chw[0] + 2, +chw[2] + 3],
                [-chw[1] + 1, +chw[0] + 2, -chw[2] + 3],
                [-chw[1] + 1, +chw[0] + 2, +chw[2] + 3],
            ]
        )
        both = move_cuboid.get_cube_corner_positions(
            move_cuboid.Pose([1, 2, 3], rot_z90)
        )
        np.testing.assert_array_almost_equal(expected_both_corners, both)

    def test_sample_goal_difficulty_1(self):
        for i in range(1000):
            goal = move_cuboid.sample_goal(difficulty=1)
            # verify the goal is valid (i.e. within the allowed ranges)
            try:
                move_cuboid.validate_goal(goal)
            except move_cuboid.InvalidGoalError as e:
                self.fail(
                    msg="Invalid goal: {}  pose is {}, {}".format(
                        e, e.position, e.orientation
                    ),
                )

            # verify the goal satisfies conditions of difficulty 1
            # always on ground
            self.assertEqual(
                goal.position[2], move_cuboid._CUBOID_HALF_SIZE[2]
            )

            # no orientation
            np.testing.assert_array_equal(goal.orientation, [0, 0, 0, 1])

    def test_sample_goal_difficulty_2(self):
        for i in range(1000):
            goal = move_cuboid.sample_goal(difficulty=2)
            # verify the goal is valid (i.e. within the allowed ranges)
            try:
                move_cuboid.validate_goal(goal)
            except move_cuboid.InvalidGoalError as e:
                self.fail(
                    msg="Invalid goal: {}  pose is {}, {}".format(
                        e, e.position, e.orientation
                    ),
                )

            # verify the goal satisfies conditions of difficulty 2
            self.assertLessEqual(goal.position[2], move_cuboid._max_height)
            self.assertGreaterEqual(goal.position[2], move_cuboid._min_height)

            # no orientation
            np.testing.assert_array_equal(goal.orientation, [0, 0, 0, 1])

    def test_sample_goal_difficulty_3(self):
        for i in range(1000):
            goal = move_cuboid.sample_goal(difficulty=3)
            # verify the goal is valid (i.e. within the allowed ranges)
            try:
                move_cuboid.validate_goal(goal)
            except move_cuboid.InvalidGoalError as e:
                self.fail(
                    msg="Invalid goal: {}  pose is {}, {}".format(
                        e, e.position, e.orientation
                    ),
                )

            # verify the goal satisfies conditions of difficulty 2
            self.assertLessEqual(goal.position[2], move_cuboid._max_height)
            self.assertGreaterEqual(goal.position[2], move_cuboid._min_height)

            # no orientation
            np.testing.assert_array_equal(goal.orientation, [0, 0, 0, 1])

    def test_sample_goal_difficulty_4(self):
        for i in range(1000):
            goal = move_cuboid.sample_goal(difficulty=4)
            # verify the goal is valid (i.e. within the allowed ranges)
            try:
                move_cuboid.validate_goal(goal)
            except move_cuboid.InvalidGoalError as e:
                self.fail(
                    msg="Invalid goal: {}  pose is {}, {}".format(
                        e, e.position, e.orientation
                    ),
                )

            # verify the goal satisfies conditions of difficulty 2
            self.assertLessEqual(goal.position[2], move_cuboid._max_height)
            self.assertGreaterEqual(goal.position[2], move_cuboid._min_height)

    def test_evaluate_state_difficulty_1(self):
        difficulty = 1
        pose_origin = move_cuboid.Pose()
        pose_trans = move_cuboid.Pose(position=[1, 2, 3])
        pose_rot = move_cuboid.Pose(
            orientation=Rotation.from_euler("z", 0.42).as_quat()
        )
        pose_both = move_cuboid.Pose(
            [1, 2, 3], Rotation.from_euler("z", 0.42).as_quat()
        )

        # needs to be zero for exact match
        cost = move_cuboid.evaluate_state(pose_origin, pose_origin, difficulty)
        self.assertEqual(cost, 0)

        # None-zero if there is translation, rotation is ignored
        self.assertNotEqual(
            move_cuboid.evaluate_state(pose_origin, pose_trans, difficulty), 0
        )
        self.assertEqual(
            move_cuboid.evaluate_state(pose_origin, pose_rot, difficulty), 0
        )
        self.assertNotEqual(
            move_cuboid.evaluate_state(pose_origin, pose_both, difficulty), 0
        )

    def test_evaluate_state_difficulty_2(self):
        difficulty = 2
        pose_origin = move_cuboid.Pose()
        pose_trans = move_cuboid.Pose(position=[1, 2, 3])
        pose_rot = move_cuboid.Pose(
            orientation=Rotation.from_euler("z", 0.42).as_quat()
        )
        pose_both = move_cuboid.Pose(
            [1, 2, 3], Rotation.from_euler("z", 0.42).as_quat()
        )

        # needs to be zero for exact match
        cost = move_cuboid.evaluate_state(pose_origin, pose_origin, difficulty)
        self.assertEqual(cost, 0)

        # None-zero if there is translation, rotation is ignored
        self.assertNotEqual(
            move_cuboid.evaluate_state(pose_origin, pose_trans, difficulty), 0
        )
        self.assertEqual(
            move_cuboid.evaluate_state(pose_origin, pose_rot, difficulty), 0
        )
        self.assertNotEqual(
            move_cuboid.evaluate_state(pose_origin, pose_both, difficulty), 0
        )

    def test_evaluate_state_difficulty_3(self):
        difficulty = 3
        pose_origin = move_cuboid.Pose()
        pose_trans = move_cuboid.Pose(position=[1, 2, 3])
        pose_rot = move_cuboid.Pose(
            orientation=Rotation.from_euler("z", 0.42).as_quat()
        )
        pose_both = move_cuboid.Pose(
            [1, 2, 3], Rotation.from_euler("z", 0.42).as_quat()
        )

        # needs to be zero for exact match
        cost = move_cuboid.evaluate_state(pose_origin, pose_origin, difficulty)
        self.assertEqual(cost, 0)

        # None-zero if there is translation, rotation is ignored
        self.assertNotEqual(
            move_cuboid.evaluate_state(pose_origin, pose_trans, difficulty), 0
        )
        self.assertEqual(
            move_cuboid.evaluate_state(pose_origin, pose_rot, difficulty), 0
        )
        self.assertNotEqual(
            move_cuboid.evaluate_state(pose_origin, pose_both, difficulty), 0
        )

    def test_evaluate_state_difficulty_4(self):
        difficulty = 4
        pose_origin = move_cuboid.Pose()
        pose_trans = move_cuboid.Pose(position=[1, 2, 3])
        pose_rot = move_cuboid.Pose(
            orientation=Rotation.from_euler("z", 0.42).as_quat()
        )
        pose_both = move_cuboid.Pose(
            [1, 2, 3], Rotation.from_euler("z", 0.42).as_quat()
        )

        # needs to be zero for exact match
        cost = move_cuboid.evaluate_state(pose_origin, pose_origin, difficulty)
        self.assertEqual(cost, 0)

        # None-zero if there is translation, rotation or both
        self.assertNotEqual(
            move_cuboid.evaluate_state(pose_origin, pose_trans, difficulty), 0
        )
        self.assertNotEqual(
            move_cuboid.evaluate_state(pose_origin, pose_rot, difficulty), 0
        )
        self.assertNotEqual(
            move_cuboid.evaluate_state(pose_origin, pose_both, difficulty), 0
        )

    def test_evaluate_state_difficulty_4_rotation_around_y(self):
        difficulty = 4
        pose_origin = move_cuboid.Pose()
        pose_rot_only_y = move_cuboid.Pose(
            orientation=Rotation.from_euler("y", 0.42).as_quat()
        )
        pose_rot_without_y = move_cuboid.Pose(
            orientation=Rotation.from_euler("yz", [0.0, 0.42]).as_quat()
        )
        pose_rot_with_y = move_cuboid.Pose(
            orientation=Rotation.from_euler("yz", [0.2, 0.42]).as_quat()
        )

        # needs to be zero for exact match
        cost = move_cuboid.evaluate_state(pose_origin, pose_origin, difficulty)
        self.assertEqual(cost, 0)

        cost_only_y = move_cuboid.evaluate_state(
            pose_origin, pose_rot_only_y, difficulty
        )
        self.assertEqual(cost_only_y, 0)

        cost_without_y = move_cuboid.evaluate_state(
            pose_origin, pose_rot_without_y, difficulty
        )
        cost_with_y = move_cuboid.evaluate_state(
            pose_origin, pose_rot_with_y, difficulty
        )
        self.assertAlmostEqual(cost_without_y, cost_with_y)

    def test_validate_goal(self):
        on_ground_height = move_cuboid._CUBOID_HALF_SIZE[2]
        yaw_rotation = Rotation.from_euler("z", 0.42).as_quat()
        full_rotation = Rotation.from_euler("zxz", [0.42, 0.1, -2.3]).as_quat()

        # test some valid goals
        try:
            move_cuboid.validate_goal(
                move_cuboid.Pose([0, 0, on_ground_height], [0, 0, 0, 1])
            )
        except Exception as e:
            self.fail("Valid goal was considered invalid because %s" % e)

        try:
            move_cuboid.validate_goal(
                move_cuboid.Pose([0.05, -0.1, on_ground_height], yaw_rotation)
            )
        except Exception as e:
            self.fail("Valid goal was considered invalid because %s" % e)

        try:
            move_cuboid.validate_goal(
                move_cuboid.Pose([-0.12, 0.0, 0.06], full_rotation)
            )
        except Exception as e:
            self.fail("Valid goal was considered invalid because %s" % e)

        # test some invalid goals

        # invalid values
        with self.assertRaises(ValueError):
            move_cuboid.validate_goal(move_cuboid.Pose([0, 0], [0, 0, 0, 1]))
        with self.assertRaises(ValueError):
            move_cuboid.validate_goal(move_cuboid.Pose([0, 0, 0], [0, 0, 1]))

        # invalid positions
        with self.assertRaises(move_cuboid.InvalidGoalError):
            move_cuboid.validate_goal(
                move_cuboid.Pose([0.3, 0, on_ground_height], [0, 0, 0, 1])
            )
        with self.assertRaises(move_cuboid.InvalidGoalError):
            move_cuboid.validate_goal(
                move_cuboid.Pose([0, -0.3, on_ground_height], [0, 0, 0, 1])
            )
        with self.assertRaises(move_cuboid.InvalidGoalError):
            move_cuboid.validate_goal(
                move_cuboid.Pose([0, 0, 0.3], [0, 0, 0, 1])
            )
        with self.assertRaises(move_cuboid.InvalidGoalError):
            move_cuboid.validate_goal(
                move_cuboid.Pose([0, 0, 0], [0, 0, 0, 1])
            )
        with self.assertRaises(move_cuboid.InvalidGoalError):
            move_cuboid.validate_goal(
                move_cuboid.Pose([0, 0, -0.01], [0, 0, 0, 1])
            )

        # valid CoM position but rotation makes it reach out of valid range
        with self.assertRaises(move_cuboid.InvalidGoalError):
            move_cuboid.validate_goal(
                move_cuboid.Pose([0, 0, on_ground_height], full_rotation)
            )


if __name__ == "__main__":
    unittest.main()
