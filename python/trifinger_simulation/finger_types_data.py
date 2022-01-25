"""Get model information for the different Finger types."""
import typing


class FingerTypesDataFormat(typing.NamedTuple):
    """Describes the format for the finger type data,
    comprising of the corresponding urdf, and the
    number of fingers.
    """

    urdf_file: str
    number_of_fingers: int


finger_types_data = {
    "fingerone": FingerTypesDataFormat("finger.urdf", 1),
    "trifingerone": FingerTypesDataFormat("trifinger.urdf", 3),
    "fingeredu": FingerTypesDataFormat("edu/fingeredu.urdf", 1),
    "trifingeredu": FingerTypesDataFormat("edu/trifingeredu.urdf", 3),
    "fingerpro": FingerTypesDataFormat("pro/fingerpro.urdf", 1),
    "trifingerpro": FingerTypesDataFormat("pro/trifingerpro.urdf", 3),
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
        ValueError: If *key* is not a valid finger type.
    """
    check_finger_type(name)
    return finger_types_data[name].number_of_fingers
