import math
import numpy as np
import time

import gym
from gym import spaces
from pybullet_fingers.sim_finger import SimFinger
from pybullet_fingers.gym_wrapper.data_logger import DataLogger
from pybullet_fingers.gym_wrapper.finger_spaces import FingerSpaces
from pybullet_fingers.gym_wrapper import utils


class FingerPush(gym.Env):
    """
    A gym environment to enable training on either the single or
    the tri-fingers robots for the task of pushing

    Args:
        control_rate_s (float): the rate at which the env step runs
        enable_visualization (bool): if the simulation env is to be
            visualized
        finger-type (str "single"/"tri"): to train on the "single"
            or the "tri" finger
        sampling_strategy (str [default]"separated"/"triangle"): the
            strategy according to which the goals are sampled
            ([default] "separated")
    """

    def __init__(self, control_rate_s, enable_visualization,
                 finger_type, sampling_strategy="separated"):
        """
        Constructor sets up the physical world parameters,
        and resets to begin training.
        """

        self.logger = DataLogger()

        if finger_type == "single":
            self.num_fingers = 1
        else:
            self.num_fingers = 3
        simulation_rate_s = 0.004
        self.steps_per_control = int(round(control_rate_s / simulation_rate_s))
        assert(abs(control_rate_s - self.steps_per_control * simulation_rate_s)
               <= 0.000001)

        self.observations_keys = [
            'joint_positions',
            'joint_velocities',
            'action_joint_positions',
            'goal_position',
            'object_position'
        ]
        self.observations_sizes = [
            3 * self.num_fingers,
            3 * self.num_fingers,
            3 * self.num_fingers,
            3,
            3
        ]

        self.spaces = FingerSpaces(num_fingers=self.num_fingers,
                                   observations_keys=self.observations_keys,
                                   observations_sizes=self.observations_sizes,
                                   separate_goals=False)

        self.finger = SimFinger(time_step=simulation_rate_s,
                                enable_visualization=enable_visualization,
                                finger_type=finger_type,
                                action_bounds=self.spaces.action_bounds,
                                sampling_strategy=sampling_strategy)

        gym.Env.__init__(self)

        self.metadata = {'render.modes': ['human']}

        self.unscaled_observation_space = \
            self.spaces.get_unscaled_observation_space()
        self.unscaled_action_space = self.spaces.get_unscaled_action_space()

        self.observation_space = self.spaces.get_scaled_observation_space()
        self.action_space = self.spaces.get_scaled_action_space()

        self.distance_threshold = 0.05
        self.epsilon_reward = 0.001

        self.finger.import_interaction_objects()
        self.finger.set_visual_goal_object_position()

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

    def _get_observation(self, action, log_observation=False):
        """
        Get the current observation from the env for the agent

        Args:
            log_observation (bool): specify whether you want to
                log the observation

        Returns:
            observation (list): comprising of the observations corresponding
                to the key values in the observation_keys
        """
        joint_positions = self.finger.observation.position
        joint_velocities = self.finger.observation.velocity
        tip_positions = self.finger.forward_kinematics(
            joint_positions)
        end_effector_position = np.concatenate(tip_positions)
        flat_goals = np.concatenate([self.goal] * self.num_fingers)

        if self.num_fingers == 1:
            flat_goals = self.goal

        end_effector_to_goal = list(np.subtract(flat_goals,
                                    end_effector_position))
        observation_dict = {}
        observation_dict['joint_positions'] = joint_positions
        observation_dict['joint_velocities'] = joint_velocities
        observation_dict['end_effector_position'] = end_effector_position
        observation_dict['end_effector_to_goal'] = end_effector_to_goal
        observation_dict['goal_position'] = self.goal
        observation_dict['object_position'] = \
            self.finger.get_block_state()[0:3]
        observation_dict['action_joint_positions'] = action

        if log_observation:
            self.logger.append(observation_dict['joint_positions'],
                               observation_dict['end_effector_position'],
                               time.time())

        observation = [v
                       for key in self.spaces.observations_keys
                       for v in observation_dict[key]]

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
        unscaled_action = utils.unscale(action, self.unscaled_action_space)
        observation = None
        for _ in range(self.steps_per_control):
            self.finger.set_action(unscaled_action, "position")
            self.finger.step_robot(observation is None)
            if observation is None:
                observation = self._get_observation(unscaled_action, True)

        key_observation = observation[
            self.spaces.key_to_index['object_position']]

        reward, done = self._compute_reward(key_observation,
                                            self.goal)
        info = {'is_success': np.float32(done)}

        scaled_observation = utils.scale(observation,
                                         self.unscaled_observation_space)
        print("reward", reward)

        return scaled_observation, reward, done, info

    def reset(self):
        """
        Episode reset

        Returns:
            the scaled to [-1;1] observation from the env after the reset
        """
        action = self.finger.reset_finger()
        self.goal = self.finger.sample_random_position_in_arena(
            height_limits=0.0425)
        self.block_position = self.finger.sample_random_position_in_arena(
            height_limits=0.0425)
        self.finger.set_goal_object_position(self.goal)
        self.finger.set_block_state(self.block_position, [0, 0, 0, 1])

        self.logger.new_episode(self.block_position, self.goal)

        return utils.scale(self._get_observation(action, True),
                           self.unscaled_observation_space)
