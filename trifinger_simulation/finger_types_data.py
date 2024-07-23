"""Get model information for the different Finger types."""

import typing


class FingerTypesDataFormat(typing.NamedTuple):
    """Describes the format for the finger type data,
    comprising of the corresponding urdf, and the
    number of fingers.
    """

    #: Path to the URDF file (relative to the URDF base directory)
    urdf_file: str

    #: Number of fingers the robot has.
    number_of_fingers: int

    #: Initial joint positions.
    initial_joint_positions: typing.Sequence[float]


finger_types_data = {
    "fingerone": FingerTypesDataFormat(
        urdf_file="finger.urdf",
        number_of_fingers=1,
        initial_joint_positions=[0, 0, 0],
    ),
    "trifingerone": FingerTypesDataFormat(
        urdf_file="trifinger.urdf",
        number_of_fingers=3,
        initial_joint_positions=[0, -0.9, -1.7] * 3,
    ),
    "fingeredu": FingerTypesDataFormat(
        urdf_file="edu/fingeredu.urdf",
        number_of_fingers=1,
        initial_joint_positions=[0, 0.9, -1.7],
    ),
    "trifingeredu": FingerTypesDataFormat(
        urdf_file="edu/trifingeredu.urdf",
        number_of_fingers=3,
        initial_joint_positions=[0, 0.9, -1.7] * 3,
    ),
    "fingerpro": FingerTypesDataFormat(
        urdf_file="pro/fingerpro.urdf",
        number_of_fingers=1,
        initial_joint_positions=[0, 0.9, -1.7],
    ),
    "trifingerpro": FingerTypesDataFormat(
        urdf_file="pro/trifingerpro.urdf",
        number_of_fingers=3,
        initial_joint_positions=[0, 0.9, -1.7] * 3,
    ),
}


def get_valid_finger_types() -> typing.KeysView[str]:
    """
    Get list of supported finger types.

    Returns:
        List of supported finger types.
    """
    return finger_types_data.keys()


def check_finger_type(name: str) -> str:
    """
    Check if *name* is a valid finger type.

    Args:
        name: Name of the finger type.

    Returns:
        The name if it is valid.

    Raises:
        ValueError: If *name* is not a valid finger type.
    """
    if name not in finger_types_data.keys():
        raise ValueError(
            "Invalid finger type '%s'.  Valid types are %s"
            % (name, finger_types_data.keys())
        )
    else:
        return name


def get_finger_urdf(name: str) -> str:
    """
    Get the name of the URDF-file with the model of the specified finger type.

    Args:
        name: Name of the finger type.

    Returns:
        The name of the URDF file in the ``robot_properties_fingers`` package.

    Raises:
        ValueError: If *name* is not a valid finger type.
    """
    check_finger_type(name)
    return finger_types_data[name].urdf_file


def get_number_of_fingers(name: str) -> int:
    """
    Get the number of fingers of the specified finger type

    Args:
        name: Name of the finger type.

    Returns:
        Number of fingers.

    Raises:
        ValueError: If *name* is not a valid finger type.
    """
    check_finger_type(name)
    return finger_types_data[name].number_of_fingers


def get_initial_joint_positions(name: str) -> typing.Sequence[float]:
    """
    Get initial joint positions of the specified finger type

    Args:
        name: Name of the finger type.

    Returns:
        Angular joint positions.

    Raises:
        ValueError: If *name* is not a valid finger type.
    """
    check_finger_type(name)
    return finger_types_data[name].initial_joint_positions
