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


def get_valid_finger_types():
    """
    Get list of supported finger types.

    Returns:
        list: List of supported finger types.
    """
    return finger_types_data.keys()


def check_finger_type(key):
    """
    Check if a key value is a valid finger type.

    Returns:
        string: The key value if it is valid
    """
    if key not in finger_types_data.keys():
        raise ValueError(
            "Invalid finger type '%s'.  Valid types are %s"
            % (key, finger_types_data.keys())
        )
    else:
        return key


def get_finger_urdf(key):
    """
    Get the name of the file with the urdf model of the finger type.

    Returns:
        string: The name of this urdf file

    Raises:
        ValueError if the key passed is an invalid finger_type
    """
    finger_type = check_finger_type(key)
    return finger_types_data[finger_type].urdf_file


def get_number_of_fingers(key):
    """
    Get the number of fingers of the finger type

    Returns:
        int: the number of fingers

    Raises:
        ValueError if the key passed is an invalid finger_type
    """
    finger_type = check_finger_type(key)
    return finger_types_data[finger_type].number_of_fingers
