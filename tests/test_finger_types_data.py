import pytest

import trifinger_simulation.finger_types_data as ftd


def test_get_valid_finger_types():
    expected_types = [
        "fingerone",
        "trifingerone",
        "fingeredu",
        "trifingeredu",
        "fingerpro",
        "trifingerpro",
    ]
    actual_types = ftd.get_valid_finger_types()

    assert sorted(actual_types) == sorted(expected_types)


def test_check_finger_type():
    for name in ftd.finger_types_data.keys():
        assert ftd.check_finger_type(name) == name

    with pytest.raises(ValueError):
        ftd.check_finger_type("invalid")


def test_get_finger_urdf():
    for name in ftd.finger_types_data.keys():
        assert (
            ftd.get_finger_urdf(name) == ftd.finger_types_data[name].urdf_file
        )

    with pytest.raises(ValueError):
        ftd.check_finger_type("invalid")


def test_get_number_of_fingers():
    for name in ftd.finger_types_data.keys():
        assert (
            ftd.get_number_of_fingers(name)
            == ftd.finger_types_data[name].number_of_fingers
        )

    with pytest.raises(ValueError):
        ftd.check_finger_type("invalid")
