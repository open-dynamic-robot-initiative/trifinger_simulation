import math
import numpy as np

from gym import spaces


class FingerSpaces:
    """
    Sets up the observation and action spaces for a finger env depending
    on the observations used.

    Args:
        num_fingers (int): The number of fingers, 1 or 3.
        observations_keys (list of strings): The keys corresponding to the
            observations used in the env
        observations_sizes (list of ints): The sizes of each of the keys
            in observations_keys in the same order.
        separate_goals (bool): Whether the 3 fingers get the same goal
            or separate goals.
    """

    def __init__(
        self,
        num_fingers,
        observations_keys,
        observations_sizes,
        separate_goals,
    ):
        self.num_fingers = num_fingers

        self.lower_bounds = {}
        self.upper_bounds = {}

        self.observations_keys = observations_keys
        self.observations_sizes = observations_sizes
        self.key_to_index = {}

        self.separate_goals = separate_goals

        assert len(self.observations_keys) == len(self.observations_sizes), (
            "Specify the size for each expected observation key."
            "And this is not being checked, but the sizes must be"
            "in the same order as the keys."
        )

        slice_start = 0
        for i in range(len(self.observations_keys)):
            self.key_to_index[self.observations_keys[i]] = slice(
                slice_start, slice_start + self.observations_sizes[i]
            )
            slice_start += self.observations_sizes[i]

        self.action_bounds = {
            "low": np.array(
                [-math.radians(70), -math.radians(70), -math.radians(160)]
                * self.num_fingers
            ),
            "high": np.array(
                [math.radians(70), 0, math.radians(-2)] * self.num_fingers
            ),
        }

        self.lower_bounds["action_joint_positions"] = self.action_bounds["low"]
        self.upper_bounds["action_joint_positions"] = self.action_bounds[
            "high"
        ]

        self.lower_bounds["end_effector_position"] = [
            -0.5,
            -0.5,
            0.0,
        ] * self.num_fingers
        self.upper_bounds["end_effector_position"] = [
            0.5,
            0.5,
            0.5,
        ] * self.num_fingers

        self.lower_bounds["joint_positions"] = [
            -math.radians(90),
            -math.radians(90),
            -math.radians(172),
        ] * self.num_fingers
        self.upper_bounds["joint_positions"] = [
            math.radians(90),
            math.radians(100),
            math.radians(-2),
        ] * self.num_fingers

        self.lower_bounds["joint_velocities"] = [-20] * 3 * self.num_fingers
        self.upper_bounds["joint_velocities"] = [20] * 3 * self.num_fingers

        self.lower_bounds["end_effector_to_goal"] = (
            [-0.5] * 3 * self.num_fingers
        )
        self.upper_bounds["end_effector_to_goal"] = (
            [0.5] * 3 * self.num_fingers
        )

        if self.separate_goals:
            self.lower_bounds["goal_position"] = [-0.5] * 3 * self.num_fingers
            self.upper_bounds["goal_position"] = [0.5] * 3 * self.num_fingers
        else:
            self.lower_bounds["goal_position"] = [-0.5] * 3
            self.upper_bounds["goal_position"] = [0.5] * 3

        self.lower_bounds["object_position"] = [-0.5, -0.5, 0.0]
        self.upper_bounds["object_position"] = [0.5, 0.5, 0.5]

    def get_unscaled_observation_space(self):
        """
        Returns the unscaled observation space corresponding
        to the observation bounds
        """
        observation_lower_bounds = [
            value
            for key in self.observations_keys
            for value in self.lower_bounds[key]
        ]
        observation_higher_bounds = [
            value
            for key in self.observations_keys
            for value in self.upper_bounds[key]
        ]
        return spaces.Box(
            low=np.array(observation_lower_bounds),
            high=np.array(observation_higher_bounds),
        )

    def get_unscaled_action_space(self):
        """
        Returns the unscaled action space according to the action bounds.
        """
        return spaces.Box(
            low=self.action_bounds["low"],
            high=self.action_bounds["high"],
            dtype=np.float32,
        )

    def get_scaled_observation_space(self):
        """
        Returns an observation space with the same size as the unscaled
        but bounded by -1s and 1s.
        """
        unscaled_observation_space = self.get_unscaled_observation_space()
        return spaces.Box(
            low=-np.ones_like(unscaled_observation_space.low),
            high=np.ones_like(unscaled_observation_space.high),
        )

    def get_scaled_action_space(self):
        """
        Returns an action space with the same size as the unscaled
        but bounded by -1s and 1s.
        """
        return spaces.Box(
            low=-np.ones(3 * self.num_fingers),
            high=np.ones(3 * self.num_fingers),
        )
