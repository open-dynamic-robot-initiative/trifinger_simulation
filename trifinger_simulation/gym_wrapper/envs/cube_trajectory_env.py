"""Gym environment for the Real Robot Challenge Phase 1 (Simulation)."""
import enum

import numpy as np
import gym

from trifinger_simulation import TriFingerPlatform
from trifinger_simulation import visual_objects
from trifinger_simulation.tasks import move_cube_on_trajectory as mct


class Initializer:
    """Base initializer for getting a trajectory."""

    def get_trajectory(self) -> mct.Trajectory:
        raise NotImplementedError()


class RandomInitializer(Initializer):
    """Initializer that samples random trajectories."""

    def get_trajectory(self):
        """Get a random trajectory."""
        return mct.sample_goal()


class FixedInitializer(Initializer):
    """Initializer that uses a fixed trajectory."""

    def __init__(self, trajectory: mct.Trajectory):
        """Initialize.

        Args:
            trajectory: The goal trajectory.

        Raises:
            Exception:  If trajectory is not valid.  See
            :meth:`mct.validate_goal` for more information.
        """
        mct.validate_goal(trajectory)
        self.trajectory = trajectory

    def get_trajectory(self):
        """Get the trajectory that was set in the constructor."""
        return self.trajectory


class ActionType(enum.Enum):
    """Different action types that can be used to control the robot."""

    #: Use pure torque commands.  The action is a list of torques (one per
    #: joint) in this case.
    TORQUE = enum.auto()
    #: Use joint position commands.  The action is a list of angular joint
    #: positions (one per joint) in this case.  Internally a PD controller is
    #: executed for each action to determine the torques that are applied to
    #: the robot.
    POSITION = enum.auto()
    #: Use both torque and position commands.  In this case the action is a
    #: dictionary with keys "torque" and "position" which contain the
    #: corresponding lists of values (see above).  The torques resulting from
    #: the position controller are added to the torques in the action before
    #: applying them to the robot.
    TORQUE_AND_POSITION = enum.auto()


class CubeTrajectoryEnv(gym.GoalEnv):
    """Gym environment for moving cubes with simulated TriFingerPro."""

    def __init__(
        self,
        initializer: Initializer,
        action_type: ActionType = ActionType.POSITION,
        step_size: int = 1,
        visualization: bool = False,
    ):
        """Initialize.

        Args:
            initializer: Initializer class for providing goal trajectories.
                See :class:`RandomInitializer` and :class:`FixedInitializer`.
            action_type (ActionType): Specify which type of actions to use.
                See :class:`ActionType` for details.
            step_size (int):  Number of actual control steps to be performed in
                one call of step().
            visualization (bool): If true, the pyBullet GUI is run for
                visualization.
        """
        # Basic initialization
        # ====================

        self.initializer = initializer
        self.action_type = action_type
        self.visualization = visualization

        if step_size < 1:
            raise ValueError("step_size cannot be less than 1.")
        self.step_size = step_size

        # will be initialized in reset()
        self.platform = None

        # Create the action and observation spaces
        # ========================================

        spaces = TriFingerPlatform.spaces

        if self.action_type == ActionType.TORQUE:
            self.action_space = spaces.robot_torque.gym
        elif self.action_type == ActionType.POSITION:
            self.action_space = spaces.robot_position.gym
        elif self.action_type == ActionType.TORQUE_AND_POSITION:
            self.action_space = gym.spaces.Dict(
                {
                    "torque": spaces.robot_torque.gym,
                    "position": spaces.robot_position.gym,
                }
            )
        else:
            raise ValueError("Invalid action_type")

        self.observation_space = gym.spaces.Dict(
            {
                "observation": gym.spaces.Dict(
                    {
                        "position": spaces.robot_position.gym,
                        "velocity": spaces.robot_velocity.gym,
                        "torque": spaces.robot_torque.gym,
                    }
                ),
                "desired_goal": spaces.object_position.gym,
                "achieved_goal": spaces.object_position.gym,
            }
        )

    def compute_reward(
        self,
        achieved_goal: mct.Position,
        desired_goal: mct.Position,
        info: dict,
    ) -> float:
        """Compute the reward for the given achieved and desired goal.

        Args:
            achieved_goal: Current position of the object.
            desired_goal: Goal trajectory of the object.
            info: An info dictionary containing a field "time_index" which
                contains the time index of the achieved_goal.

        Returns:
            The reward that corresponds to the provided achieved goal w.r.t. to
            the desired goal. Note that the following should always hold true::

                ob, reward, done, info = env.step()
                assert reward == env.compute_reward(
                    ob['achieved_goal'],
                    ob['desired_goal'],
                    info,
                )
        """
        # This is just some sanity check to verify that the given desired_goal
        # actually matches with the active goal in the trajectory.
        active_goal = np.asarray(
            mct.get_active_goal(
                self.info["trajectory"], self.info["time_index"]
            )
        )
        assert np.all(active_goal == desired_goal), "{}: {} != {}".format(
            info["time_index"], active_goal, desired_goal
        )

        return -mct.evaluate_state(
            info["trajectory"], info["time_index"], achieved_goal
        )

    def step(self, action):
        """Run one timestep of the environment's dynamics.

        When end of episode is reached, you are responsible for calling
        ``reset()`` to reset this environment's state.

        Args:
            action: An action provided by the agent (depends on the selected
                :class:`ActionType`).

        Returns:
            tuple:

            - observation (dict): agent's observation of the current
              environment.
            - reward (float): amount of reward returned after previous action.
            - done (bool): whether the episode has ended, in which case further
              step() calls will return undefined results.
            - info (dict): info dictionary containing the current time index.
        """
        if self.platform is None:
            raise RuntimeError("Call `reset()` before starting to step.")

        if not self.action_space.contains(action):
            raise ValueError(
                "Given action is not contained in the action space."
            )

        num_steps = self.step_size

        # ensure episode length is not exceeded due to step_size
        step_count_after = self.step_count + num_steps
        if step_count_after > mct.EPISODE_LENGTH:
            excess = step_count_after - mct.EPISODE_LENGTH
            num_steps = max(1, num_steps - excess)

        reward = 0.0
        for _ in range(num_steps):
            self.step_count += 1
            if self.step_count > mct.EPISODE_LENGTH:
                raise RuntimeError("Exceeded number of steps for one episode.")

            # send action to robot
            robot_action = self._gym_action_to_robot_action(action)
            t = self.platform.append_desired_action(robot_action)

            # update goal visualization
            if self.visualization:
                goal_position = mct.get_active_goal(self.info["trajectory"], t)
                self.goal_marker.set_state(goal_position, (0, 0, 0, 1))

            # Use observations of step t + 1 to follow what would be expected
            # in a typical gym environment.  Note that on the real robot, this
            # will not be possible
            self.info["time_index"] = t + 1

            # Alternatively use the observation of step t.  This is the
            # observation from the moment before action_t is applied, i.e. the
            # result of that action is not yet visible in this observation.
            #
            # When using this observation, the resulting cumulative reward
            # should match exactly the one computed during replay (with the
            # above it will differ slightly).
            #
            # self.info["time_index"] = t

            observation = self._create_observation(self.info["time_index"])

            reward += self.compute_reward(
                observation["achieved_goal"],
                observation["desired_goal"],
                self.info,
            )

        is_done = self.step_count >= mct.EPISODE_LENGTH

        return observation, reward, is_done, self.info

    def reset(self):
        """Reset the environment."""
        # hard-reset simulation
        del self.platform

        # initialize simulation
        initial_robot_position = (
            TriFingerPlatform.spaces.robot_position.default
        )
        # initialize cube at the centre
        initial_object_pose = mct.move_cube.Pose(
            position=mct.INITIAL_CUBE_POSITION
        )

        self.platform = TriFingerPlatform(
            visualization=self.visualization,
            initial_robot_position=initial_robot_position,
            initial_object_pose=initial_object_pose,
        )

        # get goal trajectory from the initializer
        trajectory = self.initializer.get_trajectory()

        # visualize the goal
        if self.visualization:
            self.goal_marker = visual_objects.CubeMarker(
                width=mct.move_cube._CUBE_WIDTH,
                position=trajectory[0][1],
                orientation=(0, 0, 0, 1),
                pybullet_client_id=self.platform.simfinger._pybullet_client_id,
            )

        self.info = {"time_index": -1, "trajectory": trajectory}

        self.step_count = 0

        return self._create_observation(0)

    def seed(self, seed=None):
        """Sets the seed for this env’s random number generator.

        .. note::

           Spaces need to be seeded separately.  E.g. if you want to sample
           actions directly from the action space using
           ``env.action_space.sample()`` you can set a seed there using
           ``env.action_space.seed()``.

        Returns:
            List of seeds used by this environment.  This environment only uses
            a single seed, so the list contains only one element.
        """
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        mct.move_cube.random = self.np_random
        return [seed]

    def _create_observation(self, t):
        robot_observation = self.platform.get_robot_observation(t)
        camera_observation = self.platform.get_camera_observation(t)
        object_observation = camera_observation.filtered_object_pose

        active_goal = np.asarray(
            mct.get_active_goal(self.info["trajectory"], t)
        )

        observation = {
            "observation": {
                "position": robot_observation.position,
                "velocity": robot_observation.velocity,
                "torque": robot_observation.torque,
            },
            "desired_goal": active_goal,
            "achieved_goal": object_observation.position,
        }

        return observation

    def _gym_action_to_robot_action(self, gym_action):
        # construct robot action depending on action type
        if self.action_type == ActionType.TORQUE:
            robot_action = self.platform.Action(torque=gym_action)
        elif self.action_type == ActionType.POSITION:
            robot_action = self.platform.Action(position=gym_action)
        elif self.action_type == ActionType.TORQUE_AND_POSITION:
            robot_action = self.platform.Action(
                torque=gym_action["torque"], position=gym_action["position"]
            )
        else:
            raise ValueError("Invalid action_type")

        return robot_action
