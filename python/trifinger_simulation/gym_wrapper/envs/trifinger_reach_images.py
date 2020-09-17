import math
import numpy as np

import gym

from trifinger_simulation.sim_finger import SimFinger
from trifinger_simulation.gym_wrapper import utils
from trifinger_simulation import (
    collision_objects,
    sample,
    finger_types_data,
    camera,
)
import skimage.io
from skimage import transform
from datetime import date, datetime
import pybullet
import os
# from scipy.misc import imresize
# scipy==1.1.0 for image resize is lossless
# newer versions- imageio


class TriFingerReachImages(gym.Env):
    """A gym environment to enable training on any of the valid robots,
    real or simulated, for the task of reaching.
    """

    def __init__(
        self,
        control_rate_s=0.02,
        representation_size=256,
        finger_type="fingerone",
        enable_visualization=False,
        max_episode_steps=None,
        viewing_angle=25 * 10,
    ):
        self.finger_type = finger_type
        self.num_fingers = finger_types_data.get_number_of_fingers(
            self.finger_type)
        self.enable_visualization = enable_visualization
        self.finger = None
        # finger will be created (and recreated) in reset
        self.object = None

        self.representation_size = representation_size
        self.lower_bounds = {}
        self.upper_bounds = {}
        self.observations_keys = [
            "representation",
        ]
        self.lower_bounds[
            "representation"] = [-np.inf] * self.representation_size
        self.upper_bounds[
            "representation"] = [+np.inf] * self.representation_size
        self.lower_bounds[
            "action_joint_torques"] = [-0.36] * 3 * self.num_fingers
        self.upper_bounds[
            "action_joint_torques"] = [+0.36] * 3 * self.num_fingers
        self.observations_sizes = [self.representation_size]
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
        self.unscaled_observation_space = gym.spaces.Box(
            low=-np.inf,
            high=+np.inf,
            shape=(64, 64, 3),
            # low=np.array(self.lower_bounds["representation"]),
            # high=np.array(self.upper_bounds["representation"])
        )
        self.unscaled_action_space = gym.spaces.Box(
            low=np.array(self.lower_bounds["action_joint_torques"]),
            high=np.array(self.upper_bounds["action_joint_torques"])
        )
        self.key_to_index = {}
        slice_start = 0
        for i in range(len(self.observations_keys)):
            self.key_to_index[self.observations_keys[i]] = slice(
                slice_start, slice_start + self.observations_sizes[i]
            )
            slice_start += self.observations_sizes[i]
        self.observation_space = gym.spaces.Box(
            low=-np.ones_like(self.unscaled_observation_space.low),
            high=np.ones_like(self.unscaled_observation_space.high),
        )
        self.action_space = gym.spaces.Box(
            low=-np.ones_like(self.unscaled_action_space.low),
            high=np.ones_like(self.unscaled_action_space.high),
        )

        self.viewing_angle = viewing_angle
        self.data_motion_zone = dict(
                radius=0.4,
                height=0.5,
            )
        self.data_format = dict(
                image_height=256,
                image_width=256,
                )

        self._start_time = datetime.now().strftime("%Y%m%d-%H%M%S-")

        self.prev_time = datetime.now()
        self.seed()
        self.reset()

        self.distance_threshold = 0.0575
        # this would translate into a reaching error of 2.5 cm from
        # the block's surface which may be a bit too highly tolerable,
        # but let's try with it for now.
        self._max_episode_steps = max_episode_steps
        self._elapsed_steps = None
        self._num_episodes = 0
        self.steps_per_control = int(
            round(control_rate_s / self.finger.time_step_s))
        assert (
            abs(
                control_rate_s - self.steps_per_control
                * self.finger.time_step_s)
            <= 0.000001
        )

    def _compute_reward(self, observation, goal):
        joint_positions = np.array(observation.position)
        try:
            assert not np.any(
                np.isnan(
                    joint_positions)), "Joint positions is NaN!"
        except AssertionError:
            self._get_state()
            skimage.io.imsave(
                'NaN_joint_positions.png', self.bgr_img_resized
            )
            raise Exception
        try:
            end_effector_positions = \
                np.asarray(self.finger.kinematics.forward_kinematics(
                    joint_positions
                ))
            assert not np.any(
                np.isnan(
                    end_effector_positions)), "End effector positions is NaN!"
        except AssertionError:
            self._get_state()
            skimage.io.imsave(
                'NaN_tip_positions.png', self.bgr_img_resized
            )
            raise Exception
        distance_to_goal = utils.compute_distance(
            end_effector_positions, goal)
        reward = -distance_to_goal
        done = self.is_success(distance_to_goal)
        return reward * self.steps_per_control, done

    def _get_state(self):
        observation_dict = {}
        multi_view_images = []
        # for theta in range(0, 360, 10):
        camera_eye_at = [
            self.data_motion_zone["radius"] * np.cos(
                math.radians(self.viewing_angle)),
            self.data_motion_zone["radius"] * np.sin(
                math.radians(self.viewing_angle)),
            self.data_motion_zone["height"],
            ]
        sim_camera = camera.Camera(
            camera_position=camera_eye_at,
            image_size=(
                self.data_format["image_height"],
                self.data_format["image_width"],
                ),
            target_position=[0, 0, 0],
            camera_up_vector=[0, 0, 1],
            field_of_view=60.0,
            near_plane_distance=0.1,
            far_plane_distance=3.1,
            physicsClientId=self.finger._pybullet_client_id,
            )
        rgb_img_array = sim_camera.get_image(
            renderer=pybullet.ER_TINY_RENDERER
            )
        rgb_img_resized = transform.resize(
            rgb_img_array, (64, 64))
        bgr_img_resized = rgb_img_resized[..., ::-1]
        self.bgr_img_resized = bgr_img_resized
        bgr_img_resized_normalized = bgr_img_resized/255.
        multi_view_images.append(
            bgr_img_resized_normalized)
        # skimage.io.imsave('0.png', self.bgr_img_resized)
        observation_dict["representation"] = bgr_img_resized_normalized
        # import ipdb; ipdb.set_trace()
        return observation_dict["representation"]

    def step(self, action):
        assert self._elapsed_steps is not None, \
            "Cannot call env.step() before calling reset()"
        action = utils.unscale(
            action, self.unscaled_action_space
        )
        finger_action = self.finger.Action(torque=action)
        state = None
        for _ in range(self.steps_per_control):
            t = self.finger.append_desired_action(finger_action)
            observation = self.finger.get_observation(t)
            state = self._get_state()
        reward, done = self._compute_reward(observation, self.goal)
        info = {"is_success": np.float32(done)}
        if self._elapsed_steps >= self._max_episode_steps:
            done = True
        self._elapsed_steps += 1
        # state = np.asarray(
        #     utils.scale(state, self.unscaled_observation_space))
        return np.asarray(state), reward, done, info

    def is_success(self, dist):
        if dist < self.distance_threshold:
            return True
        else:
            return False

    def reset(self, log=True):
        del self.finger
        del self.object

        self._elapsed_steps = 0

        self.finger = SimFinger(
            finger_type=self.finger_type,
            enable_visualization=self.enable_visualization,
        )

        action = sample.random_joint_positions(
            self.num_fingers
        )
        observation = self.finger.reset_finger_positions_and_velocities(
            action)
        target_joint_config = np.asarray(
            sample.random_joint_positions(
                self.num_fingers
            )
        )
        self.goal = self.finger.kinematics.forward_kinematics(
            target_joint_config
        )
        self.object_height_limits = 0.0325
        self.object_angle_limits = (-2 * math.pi, 2 * math.pi)
        self.object_radius_limits = (0.0, 0.15)
        self.goal = sample.random_position_in_arena(
            height_limits=self.object_height_limits,
            angle_limits=self.object_angle_limits,
            radius_limits=self.object_radius_limits)
        now_time = datetime.now()
        print(f'time elapsed {now_time - self.prev_time}')
        print(f'Goal for the current episode {self.goal}')
        self.prev_time = datetime.now()
        self.object = collision_objects.Block(
            position=self.goal,
            color_rgba=[0, 0, 0, 1],
            physicsClientId=self.finger._pybullet_client_id,
        )
        # image = utils.scale(
        #     self._get_state(), self.unscaled_observation_space)
        image = np.asarray(self._get_state())
        if log:
            self._env_log_dir = os.path.join(
            str(self._start_time) +
            str(self.finger._pybullet_client_id),
            )
            if not os.path.isdir(self._env_log_dir):
                os.makedirs(self._env_log_dir)
            skimage.io.imsave(os.path.join(
                self._env_log_dir,
                "env_at_episode_" + str(self._num_episodes)),
                self.bgr_img_resized)
        return image

    def seed(self, seed=None):
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        return [seed]
