import numpy as np
import time

import gym

from trifinger_simulation.sim_finger import SimFinger
from trifinger_simulation.gym_wrapper.data_logger import DataLogger
from trifinger_simulation.gym_wrapper.finger_spaces import FingerSpaces
from trifinger_simulation.gym_wrapper import utils
from trifinger_simulation import (
    finger_types_data,
    collision_objects,
    visual_objects,
    sample,
)


class TriFingerPush(gym.Env):
    """A gym environment to enable training on any of the valid robots,
    real or simulated, for the task of pushing.
    """

    def __init__(
        self,
        control_rate_s,
        finger_type,
        enable_visualization,
    ):
        """Intializes the constituents of the pushing environment.

        Constructor sets up the finger robot depending on the finger type,
        sets up the observation and action spaces, smoothing for
        reducing jitter on the robot, and provides for a way to synchronize
        robots being trained independently.


        Args:
            control_rate_s (float): the rate (in seconds) at which step method of the env
                will run. The actual robot controller may run at a higher rate,
                so this is used to compute the number of robot control updates
                per environment step.
            finger_type (string): Name of the finger type.  In order to get
                a list of the valid finger types, call
                :meth:`.finger_types_data.get_valid_finger_types`
            enable_visualization (bool): if the simulation env is to be
                visualized
        """

        #: an instance of the simulated robot depending on the desired
        #: robot type
        self.finger = SimFinger(
            finger_type=finger_type,
            enable_visualization=enable_visualization,
        )

        self.num_fingers = finger_types_data.get_number_of_fingers(finger_type)

        #: the number of times the same action is to be applied to
        #: the robot in one step.
        self.steps_per_control = int(
            round(control_rate_s / self.finger.time_step_s)
        )
        assert (
            abs(
                control_rate_s
                - self.steps_per_control * self.finger.time_step_s
            )
            <= 0.000001
        )

        #: the types of observations that should be a part of the environment's
        #: observed state
        self.observations_keys = [
            "joint_positions",
            "joint_velocities",
            "action_joint_positions",
            "goal_position",
            "object_position",
        ]
        #: the size of each of the observation type that is part of the
        #: observation keys (in the same order)
        self.observations_sizes = [
            3 * self.num_fingers,
            3 * self.num_fingers,
            3 * self.num_fingers,
            3,
            3,
        ]

        # sets up the observation and action spaces for the environment,
        # unscaled spaces have the custom bounds set up for each observation
        # or action type, whereas all the values in the observation and action
        # spaces lie between 1 and -1
        self.spaces = FingerSpaces(
            num_fingers=self.num_fingers,
            observations_keys=self.observations_keys,
            observations_sizes=self.observations_sizes,
            separate_goals=False,
        )

        self.unscaled_observation_space = (
            self.spaces.get_unscaled_observation_space()
        )
        self.unscaled_action_space = self.spaces.get_unscaled_action_space()

        self.observation_space = self.spaces.get_scaled_observation_space()
        self.action_space = self.spaces.get_scaled_action_space()

        #: a logger to enable logging of observations if desired
        self.logger = DataLogger()

        #: the object that has to be pushed
        self.block = collision_objects.Block()

        #: a marker to visualize where the target goal position for the episode
        #: is
        self.goal_marker = visual_objects.Marker(
            number_of_goals=1,
            goal_size=0.0325,
            initial_position=[0.19, 0.08, 0.0425],
        )

        self.reset()

    def _compute_reward(self, object_position, goal):
        """
        The reward function of the environment

        Args:
            observation (list): the observation at the
                current step
            goal (list): the desired goal for the episode

        Returns:
            the reward, and the done signal
        """
        done = False
        dist = utils.compute_distance(object_position, goal)
        reward = -dist
        done = False
        return reward, done

    def _get_state(self, observation, action, log_observation=False):
        """
        Get the current observation from the env for the agent

        Args:
            log_observation (bool): specify whether you want to
                log the observation

        Returns:
            observation (list): comprising of the observations corresponding
                to the key values in the observation_keys
        """
        joint_positions = observation.position
        joint_velocities = observation.velocity
        tip_positions = self.finger.kinematics.forward_kinematics(
            joint_positions
        )
        end_effector_position = np.concatenate(tip_positions)
        flat_goals = np.concatenate([self.goal] * self.num_fingers)

        if self.num_fingers == 1:
            flat_goals = self.goal

        end_effector_to_goal = list(
            np.subtract(flat_goals, end_effector_position)
        )

        # populate this observation dict from which you can select which
        # observation types to finally choose depending on the keys
        # used for constructing the observation space of the environment
        observation_dict = {}
        observation_dict["joint_positions"] = joint_positions
        observation_dict["joint_velocities"] = joint_velocities
        observation_dict["end_effector_position"] = end_effector_position
        observation_dict["end_effector_to_goal"] = end_effector_to_goal
        observation_dict["goal_position"] = self.goal
        observation_dict["object_position"], _ = self.block.get_state()
        observation_dict["action_joint_positions"] = action

        # returns only the observations corresponding to the keys that were
        # used for constructing the observation space
        if log_observation:
            self.logger.append(
                observation_dict["joint_positions"],
                observation_dict["end_effector_position"],
                time.time(),
            )

        observation = [
            v
            for key in self.spaces.observations_keys
            for v in observation_dict[key]
        ]

        return observation

    def step(self, action):
        """
        The env step method

        Args:
            action (list): the joint positions that have to be achieved

        Returns:
            the observation scaled to lie between [-1;1], the reward,
            the done signal, and info on if the agent was successful at
            the current step
        """
        # unscales the action to the ranges of the action space of the
        # environment explicitly (as the prediction from the network
        # lies in the range [-1;1])
        unscaled_action = utils.unscale(action, self.unscaled_action_space)

        # this is the control loop to send the actions for a few timesteps
        # which depends on the actual control rate
        finger_action = self.finger.Action(position=unscaled_action)
        state = None
        for _ in range(self.steps_per_control):
            t = self.finger.append_desired_action(finger_action)
            observation = self.finger.get_observation(t)
            if state is None:
                state = self._get_state(observation, unscaled_action, True)

        key_observation = state[self.spaces.key_to_index["object_position"]]

        reward, done = self._compute_reward(key_observation, self.goal)
        info = {"is_success": np.float32(done)}

        scaled_observation = utils.scale(
            state, self.unscaled_observation_space
        )
        print("reward", reward)

        return scaled_observation, reward, done, info

    def reset(self):
        """
        Episode reset

        Returns:
            the scaled to [-1;1] observation from the env after the reset
        """
        # resets the finger to a random position
        action = sample.feasible_random_joint_positions_for_reaching(
            self.finger, self.spaces.action_bounds
        )
        observation = self.finger.reset_finger_positions_and_velocities(action)

        #: the episode target for the agent which is sampled randomly
        #: for each episode
        self.goal = sample.random_position_in_arena(height_limits=0.0425)

        #: the position from which the object is initialized at the
        #: beginning of each episode
        self.block_position = sample.random_position_in_arena(
            height_limits=0.0425
        )

        self.goal_marker.set_state([self.goal])
        self.block.set_state(self.block_position, [0, 0, 0, 1])

        # logs relevant information for replayability
        self.logger.new_episode(self.block_position, self.goal)

        return utils.scale(
            self._get_state(observation, action, True),
            self.unscaled_observation_space,
        )
