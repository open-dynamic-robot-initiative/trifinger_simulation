"""Gym environment for the Real Robot Challenge Phase 1 (Simulation)."""
import enum

import numpy as np
import gym

import trifinger_simulation
from trifinger_simulation.tasks import move_cube


class RandomInitializer:
    """Initializer that samples random initial states and goals."""

    def __init__(self, difficulty):
        """Initialize.

        Args:
            difficulty (int):  Difficulty level for sampling goals.
        """
        self.difficulty = difficulty

    def get_initial_state(self):
        """Get a random initial object pose (always on the ground)."""
        return move_cube.sample_goal(difficulty=-1)

    def get_goal(self):
        """Get a random goal depending on the difficulty."""
        return move_cube.sample_goal(difficulty=self.difficulty)


class FixedInitializer:
    """Initializer that uses fixed values for initial pose and goal."""

    def __init__(self, difficulty, initial_state, goal):
        """Initialize.

        Args:
            difficulty (int):  Difficulty level of the goal.  This is still
                needed even for a fixed goal, as it is also used for computing
                the reward (the cost function is different for the different
                levels).
            initial_state (move_cube.Pose):  Initial pose of the object.
            goal (move_cube.Pose):  Goal pose of the object.

        Raises:
            Exception:  If initial_state or goal are not valid.  See
            :meth:`move_cube.validate_goal` for more information.
        """
        move_cube.validate_goal(initial_state)
        move_cube.validate_goal(goal)
        self.difficulty = difficulty
        self.initial_state = initial_state
        self.goal = goal

    def get_initial_state(self):
        """Get the initial state that was set in the constructor."""
        return self.initial_state

    def get_goal(self):
        """Get the goal that was set in the constructor."""
        return self.goal


class ActionType(enum.Enum):
    """Different action types that can be used to control the robot.

    The different types are:
        TORQUE: Use pure torque commands.  The action is a list of torques (one
            per joint) in this case.
        POSITION: Use joint position commands.  The action is a list of angular
            joint positions (one per joint) in this case.  Internally a PD
            controller is executed for each action to determine the torques
            that are applied to the robot.
        TORQUE_AND_POSITION: Use both torque and position commands.  In this
            case the action is a dictionary with keys "torque" and "position"
            which contain the corresponding lists of values (see above).  The
            torques resulting from the position controller are added to the
            torques in the action before applying them to the robot.

    """

    TORQUE = enum.auto()
    POSITION = enum.auto()
    TORQUE_AND_POSITION = enum.auto()


class CubeEnv(gym.GoalEnv):
    """Gym environment for moving cubes with simulated TriFingerPro."""

    def __init__(
        self,
        initializer,
        action_type=ActionType.POSITION,
        frameskip=1,
        visualization=False,
    ):
        """Initialize.

        Args:
            initializer: Initializer class for providing initial cube pose and
                goal pose.  See RandomInitializer and FixedInitializer.
            action_type (ActionType): Specify which type of actions to use.
                See ActionType for details.
            frameskip (int):  Number of actual control steps to be performed in
                one call of step().
            visualization (bool): If true, the pyBullet GUI is run for
                visualization.
        """
        # Basic initialization
        # ====================

        self.initializer = initializer
        self.action_type = action_type
        self.visualization = visualization

        # TODO: The name "frameskip" makes sense for an atari environment but
        # not really for our scenario.  The name is also misleading as
        # "frameskip = 1" suggests that one frame is skipped while it actually
        # means "do one step per step" (i.e. no skip).
        if frameskip < 1:
            raise ValueError("frameskip cannot be less than 1.")
        self.frameskip = frameskip

        # will be initialized in reset()
        self.platform = None

        # Create the action and observation spaces
        # ========================================

        n_joints = 9
        n_fingers = 3
        max_torque_Nm = 0.36
        max_velocity_radps = 20  # FIXME does this value make sense?

        torque_space = gym.spaces.Box(
            low=np.full(n_joints, -max_torque_Nm, dtype=np.float32),
            high=np.full(n_joints, max_torque_Nm, dtype=np.float32),
        )

        position_space = gym.spaces.Box(
            low=np.array([-0.9, -1.57, -2.7] * n_fingers, dtype=np.float32),
            high=np.array([1.4, 1.57, 2.7] * n_fingers, dtype=np.float32),
        )

        velocity_space = gym.spaces.Box(
            low=np.full(n_joints, -max_velocity_radps, dtype=np.float32),
            high=np.full(n_joints, max_velocity_radps, dtype=np.float32),
        )

        object_state_space = gym.spaces.Dict(
            {
                "position": gym.spaces.Box(
                    low=np.array([-0.3, -0.3, 0], dtype=np.float32),
                    high=np.array([0.3, 0.3, 0.3], dtype=np.float32),
                ),
                "orientation": gym.spaces.Box(
                    low=-np.ones(4, dtype=np.float32),
                    high=np.ones(4, dtype=np.float32),
                ),
            }
        )

        if self.action_type == ActionType.TORQUE:
            self.action_space = torque_space
        elif self.action_type == ActionType.POSITION:
            self.action_space = position_space
        elif self.action_type == ActionType.TORQUE_AND_POSITION:
            self.action_space = gym.spaces.Dict(
                {"torque": torque_space, "position": position_space}
            )
        else:
            raise ValueError("Invalid action_type")

        self.observation_space = gym.spaces.Dict(
            {
                "observation": gym.spaces.Dict(
                    {
                        "position": position_space,
                        "velocity": velocity_space,
                        "torque": torque_space,
                    }
                ),
                "desired_goal": object_state_space,
                "achieved_goal": object_state_space,
            }
        )

    def compute_reward(self, achieved_goal, desired_goal, info):
        return -move_cube.evaluate_state(
            move_cube.Pose.from_dict(desired_goal),
            move_cube.Pose.from_dict(achieved_goal),
            info["difficulty"],
        )

    def step(self, action):
        if self.platform is None:
            raise RuntimeError("Call `reset()` before starting to step.")

        if not self.action_space.contains(action):
            raise ValueError("Given action is not contained in the action space.")

        num_steps = self.frameskip

        # ensure episode length is not exceeded due to frameskip
        step_count_after = self.step_count + num_steps
        if step_count_after > move_cube.episode_length:
            excess = step_count_after - move_cube.episode_length
            num_steps = max(1, num_steps - excess)

        reward = 0.0
        for _ in range(num_steps):
            self.step_count += 1
            if self.step_count > move_cube.episode_length:
                raise RuntimeError("Exceeded number of steps for one episode.")

            # construct robot action depending on action type
            if self.action_type == ActionType.TORQUE:
                robot_action = self.platform.Action(torque=action)
            elif self.action_type == ActionType.POSITION:
                robot_action = self.platform.Action(position=action)
            elif self.action_type == ActionType.TORQUE_AND_POSITION:
                robot_action = self.platform.Action(
                    torque=action["torque"], position=action["position"]
                )
            else:
                raise ValueError("Invalid action_type")

            # send action to robot
            t = self.platform.append_desired_action(robot_action)

            # Use observations of step t + 1 to follow what would be expected
            # in a typical gym environment.  Note that on the real robot, this
            # will not be possible
            robot_observation = self.platform.get_robot_observation(t + 1)
            object_observation = self.platform.get_object_pose(t + 1)

            observation = {
                "observation": {
                    "position": robot_observation.position,
                    "velocity": robot_observation.velocity,
                    "torque": robot_observation.torque,
                },
                "desired_goal": self.goal,
                "achieved_goal": {
                    "position": object_observation.position,
                    "orientation": object_observation.orientation,
                },
            }

            reward += self.compute_reward(
                observation["achieved_goal"], observation["desired_goal"], self.info
            )

        is_done = self.step_count == move_cube.episode_length

        return observation, reward, is_done, self.info

    def reset(self):
        # reset simulation
        # TODO use pybullet.resetSimulation() instead of recreating everything
        del self.platform
        self.platform = trifinger_simulation.TriFingerPlatform(
            visualization=self.visualization,
            initial_object_pose=self.initializer.get_initial_state(),
        )

        goal = self.initializer.get_goal()
        self.goal = {"position": goal.position, "orientation": goal.orientation}
        self.info = {"difficulty": self.initializer.difficulty}

        self.step_count = 0

    def seed(self, seed=None):
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        move_cube.random = self.np_random
        return [seed]
