from trifinger_simulation.sim_finger import int_to_rgba, SimFinger

import numpy as np


def test_int_to_rgba():
    assert int_to_rgba(0x000000) == (0.0, 0.0, 0.0, 1.0)
    assert int_to_rgba(0xFFFFFF) == (1.0, 1.0, 1.0, 1.0)
    assert int_to_rgba(0x006C66) == (0, 108 / 255, 102 / 255, 1.0)

    assert int_to_rgba(0x006C66, alpha=42) == (
        0,
        108 / 255,
        102 / 255,
        42 / 255,
    )


def test_SimFinger_sanitise_torqes():
    max_torque = 0.5

    # without velocity damping
    t = SimFinger._sanitise_torques(
        [-0.2, 0.4], max_torque, 0, np.array([0, 0])
    )
    np.testing.assert_array_equal(t, [-0.2, 0.4])

    t = SimFinger._sanitise_torques(
        [-1.2, 0.4], max_torque, 0, np.array([0, 0])
    )
    np.testing.assert_array_equal(t, [-0.5, 0.4])

    t = SimFinger._sanitise_torques(
        [-0.2, 0.6], max_torque, 0, np.array([0, 0])
    )
    np.testing.assert_array_equal(t, [-0.2, 0.5])

    t = SimFinger._sanitise_torques(
        [-0.8, 0.6], max_torque, 0, np.array([0, 0])
    )
    np.testing.assert_array_equal(t, [-0.5, 0.5])

    # with velocity damping
    t = SimFinger._sanitise_torques(
        [0, 0], max_torque, 0.2, np.array([1, -0.5])
    )
    np.testing.assert_array_almost_equal(t, [-0.2, 0.1])

    t = SimFinger._sanitise_torques([0, 0], max_torque, 1, np.array([1, -0.6]))
    np.testing.assert_array_almost_equal(t, [-0.5, 0.5])

    t = SimFinger._sanitise_torques(
        [0.1, 0.2], max_torque, 0.2, np.array([1, -0.5])
    )
    np.testing.assert_array_almost_equal(t, [-0.1, 0.3])

    t = SimFinger._sanitise_torques(
        [1, -1], max_torque, 0.2, np.array([1, -0.5])
    )
    np.testing.assert_array_almost_equal(t, [0.3, -0.4])
