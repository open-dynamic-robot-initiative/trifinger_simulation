import numpy as np
import random
import math


def random_position_in_arena(
    height_limits=(0.05, 0.15),
    angle_limits=(-2 * math.pi, 2 * math.pi),
    radius_limits=(0.0, 0.15),
):
    """
    Set a new position in the arena for the interaction object, which
    the finger has to reach.

    Args:
        height_limits: the height range to sample from, or
            a fixed height
        angle_limits: the range of angles to sample from
        radius_limits: distance range from the centre of the
            arena at which a sampled point can lie

    Returns:
        The random position of the target set in the arena.
    """
    angle = random.uniform(*angle_limits)
    radial_distance = max(radius_limits) * np.sqrt(random.random())

    if isinstance(height_limits, (int, float)):
        height_z = height_limits
    else:
        height_z = random.uniform(*height_limits)

    object_position = [
        radial_distance * math.cos(angle),
        radial_distance * math.sin(angle),
        height_z,
    ]

    return object_position


def random_joint_positions(
    number_of_fingers,
    lower_bounds=[-math.radians(30), -math.radians(60), -math.radians(100)],
    upper_bounds=[math.radians(30), math.radians(60), math.radians(2)],
):
    """Sample a random joint configuration for each finger.

    Args:
        number_of_fingers (int): specify if positions are to be sampled for
            joints of 1 finger, or 3 fingers
        lower_bounds: List of lower position bounds for upper, middle and lower
            joint of a single finger.  The same values will be used for all
            fingers if number_of_fingers > 1.  Unit: radian.
        upper_bounds: Upper position bounds of the joints.  See lower_bounds.

    Returns:
        Flat list of joint positions.
    """
    list_to_return = [
        random.uniform(lower, upper)
        for i in range(number_of_fingers)
        for lower, upper in zip(lower_bounds, upper_bounds)
    ]
    return list_to_return


def feasible_random_joint_positions_for_reaching(
    finger, action_bounds, sampling_strategy="separated"
):
    """
    Sample random joint configuration with low risk of collisions.

    In case of only one finger, this just calls
    random_joint_positions().

    In case of three fingers, the sampling strategy depends on
    self.sampling_strategy.

    Args:
        finger (SimFinger): A SimFinger object
        action_bounds (dict): The limits of the action space used by the
            policy network.  Has to contain keys "low" and "high" with lists of
            limit values.
        sampling_strategy (string):  Strategy with which positions for the
            three fingers are sampled. Unused when using the single finger. Has
            to be one of the following values:

            - "separated": Samples for each finger a tip position somewhere
                  in this fingers section of the workspace.  This should
                  result in target positions that minimize the risk of
                  collisions between the fingers.
            - "uniform": Samples for each finger a position uniformly over the
                  whole joint range.
            - "triangle": Samples a position somewhere in the workspace and
                  places the tips of the free fingers around it with fixed
                  distance.

    Returns:
        Flat list of joint angles.
    """
    if sampling_strategy == "uniform":
        return random_joint_positions(finger.number_of_fingers)

    elif sampling_strategy == "triangle":
        # this sampling strategy is deprecated (for now)
        if finger.number_of_fingers == 1:
            raise RuntimeError(
                "Sampling strategy 'triangle' cannot"
                " be used with a single finger."
            )
        random_position = random_position_in_arena()
        tip_positions = get_tip_positions_around_position(
            finger.number_of_fingers, random_position
        )
        joint_positions = finger.pybullet_inverse_kinematics(tip_positions)
        # The inverse kinematics is _very_ inaccurate, but as we anyway
        # are sampling random positions, we don't care so much for some
        # random deviation.  The placement of the goals for the single
        # fingers relative to each other should more or less be
        # preserved.
        return joint_positions

    elif sampling_strategy == "separated":

        def sample_point_in_angle_limits():
            while True:
                joint_pos = np.random.uniform(
                    low=[-np.pi / 2, np.deg2rad(-77.5), np.deg2rad(-172)],
                    high=[np.pi / 2, np.deg2rad(257.5), np.deg2rad(-2)],
                )
                tip_pos = finger.kinematics.forward_kinematics(
                    np.concatenate(
                        [joint_pos for i in range(finger.number_of_fingers)]
                    ),
                )[0]
                dist_to_center = np.linalg.norm(tip_pos[:2])
                angle = np.arccos(tip_pos[0] / dist_to_center)
                if (
                    (np.pi / 6 < angle < 5 / 6 * np.pi)
                    and (tip_pos[1] > 0)
                    and (0.02 < dist_to_center < 0.2)
                    and np.all((action_bounds["low"])[0:3] < joint_pos)
                    and np.all((action_bounds["high"])[0:3] > joint_pos)
                ):
                    return joint_pos

        joint_positions = np.concatenate(
            [
                sample_point_in_angle_limits()
                for i in range(finger.number_of_fingers)
            ]
        )

        return joint_positions

    else:
        raise ValueError(
            "Invalid sampling strategy '{}'".format(sampling_strategy)
        )


def get_tip_positions_around_position(number_of_fingers, position):
    """
    Compute finger tip positions close to the given target position

    For single finger, the tip position will be the same as the given
    position.  For the TriFinger, the tips of the three fingers will be
    placed around it with some distance to avoid collision.

    Args:
        number_of_fingers (int): tips of 1 finger or 3 fingers
        position (array-like): The target x,y,z-position.

    Returns:
        tip_positions (list of array-like): List with one target position
            for each finger tip (each position given as a (x, y, z) tuple).
    """
    position = np.array(position)
    if number_of_fingers == 1:
        return [position]
    elif number_of_fingers == 3:
        angle = np.deg2rad(-120)
        rot_120 = np.array(
            [
                [np.cos(angle), -np.sin(angle), 0],
                [np.sin(angle), np.cos(angle), 0],
                [0, 0, 1],
            ]
        )
        displacement = np.array([0.05, 0, 0])
        tip1 = position + displacement
        displacement = rot_120 @ displacement
        tip2 = position + displacement
        displacement = rot_120 @ displacement
        tip3 = position + displacement

        return [tip1, tip2, tip3]
    else:
        raise ValueError("Invalid number of fingers")
