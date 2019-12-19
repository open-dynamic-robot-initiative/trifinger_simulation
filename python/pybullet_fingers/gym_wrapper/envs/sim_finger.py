#!/usr/bin/env python3
# -------------------------------------------------------------------------------------------------
# The documentation in this code is heavily derived from the official documentation of PyBullet at
# https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit# among other scattered sources.
# -------------------------------------------------------------------------------------------------
# TODO: Experimental! For testing gym dependency and structure.
import os
import time
import math
import random
import numpy as np

import pybullet
import pybullet_data

import rospkg

import robot_interfaces

import gym
from gym import error, spaces, utils
from gym.utils import seeding


class Finger(gym.Env):
    """
    A simulation environment for the single finger robot.
    This environment is based on PyBullet, the official Python wrapper around the
    Bullet-C API.
    :param time_step: It is the time between two simulation steps. Defaults to
    1./240. Don't set this to be larger than 1./60.
    :type time_step: float
    :param visual_debugging: Set this to 'True' for a GUI interface to
    the simulation.
    :type visual_debugging: bool
    :param finger_tip: Define the link index of the end-effector. Set this looking
    at the output of print_link_information.
    :type finger_tip: int
    :param revolute_joint_ids: Define the range of movable joints to which control
    commands will be sent. Set this looking at the output of print_joint_information.
    :type revolute_joint_ids: class 'range'
    :param finger_link_ids: Define the range of links that corresponds to the
    upper, middle, and lower links of the finger. Set this looking at the output
    of print_link_information.
    :type finger_link_ids: class 'range'
    :param action_repetitions: The number of times the simulation will be stepped
    for a single control command.
    :type action_repetitions: int
    :param max_motor_torque: The maximum allowed torque that can be applied to
    any of the joint motors.
    :type max_motor_torque: float
    :param buffer_size: The size of the buffer for which actions are stored. 1
    by design.
    :type buffer_size: int
    """

    metadata = {'render.modes': ['human']}

    def __init__(self, time_step=0.001, visual_debugging=True):
        """
        Constructor, initializes the physical world we will work in.
        """
        self.visual_debugging = visual_debugging
        self.time_step = time_step
        self.finger_tip = 5
        self.revolute_joint_ids = range(2, 5)
        self.finger_link_ids = range(2, 5)
        self.position_gain = 200
        self.velocity_gain = 20
        self.max_motor_torque = 0.36
        self.buffer_size = 1

        self.connect_to_simulation()
        pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.make_physical_world()

    def connect_to_simulation(self):
        """
        Connect to the GUI physics server for visual debugging via a GUI
        interface, or enbable DIRECT connection.

        .. note::
           Both GUI and DIRECT physics servers (the server) execute the
           simulation and rendering in the same process as PyBullet(the client).
           To connect to a physics server in a different process on the same
           machine, or on a remote machine, use SHARED_MEMORY, UDP, or
           TCP networking.

           In GUI connection mode, use ctrl or alt with mouse scroll to adjust
           the view of the camera.
        """

        if self.visual_debugging:
            pybullet.connect(pybullet.GUI)
        else:
            pybullet.connect(pybullet.DIRECT)

    def make_physical_world(self):
        """
        Set parameters of the world such as gravity, time for which a
        simulation step should run etc, and load the finger and the
        interaction objects.
        """

        pybullet.setGravity(0, 0, -9.81)
        pybullet.setTimeStep(self.time_step)

        plane_id = pybullet.loadURDF("plane.urdf", [0, 0, 0])
        self.finger_id = self.import_finger_model()
        self.change_robot_properties()
        self.import_non_convex_shapes()
        self.import_interaction_objects()

    def import_finger_model(self):
        """
        Load a physics model of the finger from its urdf.

        ..note::
          use bitwise | instead of logical or as mentioned in the official
          documentation for the urdf flags.

        :return: Unique id of the body specified in the URDF (non-negative if
        the URDF can be loaded).
        :rtype: int
        """

        finger_urdf_path = os.path.join(rospkg.RosPack().get_path(
            "robot_properties_manipulator"), "urdf", "finger_with_stage.urdf")
        finger_base_position = [0, 0, 0.05]
        finger_base_orientation = pybullet.getQuaternionFromEuler([0, 0, 0])

        finger_id = pybullet.loadURDF(
            fileName=finger_urdf_path,
            basePosition=finger_base_position,
            baseOrientation=finger_base_orientation,
            useFixedBase=1,
            flags=pybullet.URDF_USE_INERTIA_FROM_FILE | pybullet.URDF_USE_SELF_COLLISION)

        return finger_id

    def import_non_convex_shapes(self):
        """
        Imports the non-convex arena (stage).
        """
        stage_mesh = os.path.join(
            rospkg.RosPack().get_path("robot_properties_manipulator"),
            "meshes/stl",
            "Stage_simplified.stl")
        stage_id = pybullet.createCollisionShape(
            shapeType=pybullet.GEOM_MESH,
            fileName=stage_mesh,
            flags=pybullet.GEOM_FORCE_CONCAVE_TRIMESH)
        stage_position = [0, 0, 0.05]
        stage_orientation = pybullet.getQuaternionFromEuler([0, 0, 0])
        stage = pybullet.createMultiBody(
            baseCollisionShapeIndex=stage_id,
            baseVisualShapeIndex=-1,
            basePosition=stage_position,
            baseOrientation=stage_orientation)

    def import_interaction_objects(self):
        """
        Import any object that the finger interacts/has to interact with.
        """
        block_position = [0.15, 0., 0.08]
        block_size = [0.03, 0.03, 0.03]
        self.block_mass = 0.25

        self.block_id = pybullet.createCollisionShape(
            shapeType=pybullet.GEOM_BOX, halfExtents=block_size)
        self.block = pybullet.createMultiBody(
            baseCollisionShapeIndex=self.block_id,
            basePosition=block_position,
            baseMass=self.block_mass)

    def change_robot_properties(self):
        """
        To change properties of the robot such as its mass, friction, damping,
        maximum joint velocities etc.
        """
        for link_id in self.finger_link_ids:
            pybullet.changeDynamics(
                bodyUniqueId=self.finger_id,
                linkIndex=link_id,
                maxJointVelocity=5,
                restitution=1,
                jointDamping=0,
                lateralFriction=0,
                spinningFriction=0,
                rollingFriction=0,
                linearDamping=0,
                angularDamping=0,
                contactStiffness=0,
                contactDamping=0)

    def print_link_information(self):
        """
        Get the link indices along with their names as defined in the urdf.

        ..note::
          Source: https://pybullet.org/Bullet/phpBB3/viewtopic.php?t=12728.

          (Output for reference)
          0 finger_base_link
          1 finger_upper_holder_link
          2 finger_upper_link
          3 finger_middle_link
          4 finger_lower_link
          5 finger_tip_link
        """

        link_name_to_index = {
            pybullet.getBodyInfo(
                self.finger_id)[0].decode('UTF-8'): -1, }

        for id in range(pybullet.getNumJoints(self.finger_id)):
            link_name = pybullet.getJointInfo(self.finger_id, id)[
                12].decode('UTF-8')
            link_name_to_index[link_name] = id
            print(link_name_to_index[link_name], link_name)

    def print_joint_information(self):
        """
        Get the joint indices along with their names as defined in the urdf.

        ..note::
          Source: https://pybullet.org/Bullet/phpBB3/viewtopic.php?t=12728.

          (Output for reference)
          0 base_to_finger
          1 finger_base_to_holder
          2 finger_base_to_upper_joint
          3 finger_upper_to_middle_joint
          4 finger_middle_to_lower_joint
          5 finger_lower_to_tip_joint
        """

        link_name_to_index = {
            pybullet.getBodyInfo(
                self.finger_id)[0].decode('UTF-8'): -1, }

        for id in range(pybullet.getNumJoints(self.finger_id)):
            joint_name = pybullet.getJointInfo(self.finger_id, id)[
                1].decode('UTF-8')
            link_name_to_index[joint_name] = id
            print(link_name_to_index[joint_name], joint_name)

    def disable_default_motors(self):
        """
        To disable the high friction velocity motors created by
        default at all revolute and prismatic joints while loading them from
        the urdf.
        """

        pybullet.setJointMotorControlArray(bodyUniqueId=self.finger_id,
                                           jointIndices=self.revolute_joint_ids,
                                           controlMode=pybullet.VELOCITY_CONTROL,
                                           targetVelocities=[0] * len(self.revolute_joint_ids),
                                           forces=[0] * len(self.revolute_joint_ids))

    def set_target_in_arena_randomly(self):
        """
        Set a new position in the arena for the interaction object, which
        the finger has to reach.

        :return: The random position of the target set in the arena.
        :rtype: list of 3 floats
        """

        angle_x = random.uniform(-2 * math.pi, 2 * math.pi)
        angle_y = random.uniform(-2 * math.pi, 2 * math.pi)
        radial_distance = random.uniform(0, 0.22)

        object_position = [
            radial_distance *
            math.cos(angle_x),
            radial_distance *
            math.sin(angle_y),
            0.08]
        return object_position

    def display_object_at_target(self, object_position):
        """
        Remove the older block and display a new one at the new position.

        :param object_position: Position in the xyz coordinate
        system at which the object is to be displayed.
        :type object_position: vec3/list of 3 floats
        """

        pybullet.removeBody(self.block)
        self.block = pybullet.createMultiBody(
            baseCollisionShapeIndex=self.block_id,
            basePosition=object_position,
            baseMass=self.block_mass)

    def inverse_kinematics(self, target_position):
        """
        Compute the joint angular positions needed to get to reach the block.

        :param target_position: (vec3/list of 3 floats) Target position in the
        xyz coordinate system of the end-effector.
        :type target_position: vec3/list of 3 floats
        :return: The required positions of the joints to reach the target position.
        :rtype: list of 3 floats
        """

        at_target_threshold = 0.001
        joint_pos = list(
            pybullet.calculateInverseKinematics(
                bodyUniqueId=self.finger_id,
                endEffectorLinkIndex=self.finger_tip,
                targetPosition=target_position,
                residualThreshold=at_target_threshold))

        return joint_pos

    def append_desired_action(self, action):
        """
        Pass an action on which safety checks will be performed and then the
        action will be sent to the simulated finger.

        .. note::
        For now, only torques as actions are valid.

        :param action: The structure consisting of the torques to be applied.
        :type action: class 'robot_interfaces.py_finger_types.Action'
        :return time_index: The current time index in the buffer of length 1. 1.
        :rtype time_index: int
        """
        command = self._apply_action_uninitialized(action)
        self.torque_control(command)
        time_index = self.buffer_size
        return time_index

    def apply_action_uninitialized(self, action):
        """
        Extract the torque commands from the applied action structure.

        :param action: The structure consisting of the torques to be applied.
        :type action: class 'robot_interfaces.py_finger_types.Action'
        :return list(action.torque): The torque field of Action.
        :rtype list(action.torque): list of floats.
        """

        return list(action.torque)

    def get_observation(self, time_index):
        """
        Get the observation (position/velocity/torque) of the joints
        corresponding to the current time index.

        :param time_index: The current time index in the buffer of length 1. 1.
        :type time_index: int
        :return observation: The Observation structure consisting of the position,
        velocity, and torques of the joints.
        :rtype observation: class 'robot_interfaces.py_finger_types.Observation'
        """

        if time_index == 1:
            observation = robot_interfaces.finger.Observation
            current_joint_states = pybullet.getJointStates(
                self.finger_id, self.revolute_joint_ids)

            observation.position = [
                current_joint_states[0][0],
                current_joint_states[1][0],
                current_joint_states[2][0]]
            observation.velocity = [
                current_joint_states[0][1],
                current_joint_states[1][1],
                current_joint_states[2][1]]
            observation.torque = [
                current_joint_states[0][3],
                current_joint_states[1][3],
                current_joint_states[2][3]]

        elif time_index > 1:
            print("You are trying to access information from an action that is yet to occur in the future. Valid argument to get_observation is 1 ONLY.")
        elif time_index < 1:
            print(
                "We do not store information from the past. Valid argument to get_observation is 1 ONLY.")

        return observation

    def _apply_action(self, target_position, control_mode):
        """
        Specify the target position for the finger tip and the method of control
        (torque or position) to obtain the joint positions for reaching there
        and then applying the torque or the position control commands.

        :param target_position: a valid target position to which you want to send
        the finger-tip.
        :type target_position: list of 3 floats
        :param control_mode: specify if you want to control the finger using
        torque control or position control
        :type control_mode: string, valid strings = {"position", "torque"}
        """

        joint_positions = self.inverse_kinematics(
            target_position)

        if control_mode == "torque":
            torque_commands = self.compute_pd_control_torques(joint_positions)
            self.torque_control(torque_commands)
        elif control_mode == "position":
            self.position_control(joint_positions)
        else:
            # print("Invalid control_mode, enter either " f'"position"' " or " f'"torque"'".")
            # python 3.5 :/
            print('Invalid control_mode, enter either "position" or "torque".')

    def compute_pd_control_torques(self, joint_positions):
        """
        Specify the joint positions that the motors have to reach, and compute
        the torques to be applied to reach these positions using PD control.

        :param joint_positions: The desired joint positions.
        :type joint_positions: list of 3 floats
        :return The torques to be sent to the joints of the finger in order to
        reach the specified joint_positions.
        :rtype: list of 3 floats
        """

        current_joint_states = pybullet.getJointStates(
            self.finger_id, self.revolute_joint_ids)
        current_position = [
            current_joint_states[0][0],
            current_joint_states[1][0],
            current_joint_states[2][0]]
        current_velocity = [
            current_joint_states[0][1],
            current_joint_states[1][1],
            current_joint_states[2][1]]

        position_error = list(np.subtract(joint_positions, current_position))

        kp = [self.position_gain] * len(self.revolute_joint_ids)
        kd = [self.velocity_gain] * len(self.revolute_joint_ids)

        position_feedback = list(np.multiply(kp, position_error))
        velocity_feedback = list(np.multiply(kd, current_velocity))

        # joint_torques = list(np.clip(np.add(position_feedback, velocity_feedback), -self.max_motor_torque, self.max_motor_torque))
        joint_torques = list(np.subtract(position_feedback, velocity_feedback))

        return joint_torques

    def torque_control(self, torque_commands):
        """
        Send torque commands to the motors directly.

        :param torque_commands: The forces to be sent to the movable joints of
        the motor.
        :type torque_commands: list of 3 floats
        """

        pybullet.setJointMotorControlArray(
            bodyUniqueId=self.finger_id,
            jointIndices=self.revolute_joint_ids,
            controlMode=pybullet.TORQUE_CONTROL,
            forces=torque_commands)

    def position_control(self, position_commands):
        """
        Perform position control on the finger (CONTROL_MODE_POSITION_VELOCITY_PD)

        :param joint_positions: The desired target position of the joints.
        :type joint_positions: list of floats
        """

        pybullet.setJointMotorControlArray(
            bodyUniqueId=self.finger_id,
            jointIndices=self.revolute_joint_ids,
            controlMode=pybullet.POSITION_CONTROL,
            targetPositions=position_commands,
            targetVelocities=[0] * len(
                self.revolute_joint_ids))
        #forces=[self.max_motor_torque] * len(self.revolute_joint_ids)
        current_joint_states = pybullet.getJointStates(
            self.finger_id, self.revolute_joint_ids)
        print([current_joint_states[0][3],
               current_joint_states[1][3],
               current_joint_states[2][3]])

    def _step_simulation(self, mode="human"):
        """
        Step the simulation to go to the next world state.
        """
        pybullet.stepSimulation()
