#!/usr/bin/env python3
import unittest
import numpy as np

from trifinger_simulation.sim_finger import SimFinger
from trifinger_simulation import sample


class TestResetJoints(unittest.TestCase):
    """
    This test verifies that the state of the finger(s) gets reset correctly.

    So, all the 1DOF joints of the finger(s) should be at the *exact* positions
    and have the *exact* same velocities to which we want the joints to get
    reset to.
    """

    def test_reproduce_reset_state(self):
        """
        Send hundred states (positions + velocities) to all the 1DOF joints
        of the fingers and assert they exactly reach these states.
        """
        finger = SimFinger(
            finger_type="fingerone",
        )

        for _ in range(100):
            state_positions = sample.random_joint_positions(
                finger.number_of_fingers
            )
            state_velocities = [pos * 10 for pos in state_positions]

            reset_state = finger.reset_finger_positions_and_velocities(
                state_positions, state_velocities
            )

            reset_positions = reset_state.position
            reset_velocities = reset_state.velocity

            np.testing.assert_array_equal(
                reset_positions, state_positions, verbose=True
            )
            np.testing.assert_array_equal(
                reset_velocities, state_velocities, verbose=True
            )


if __name__ == "__main__":
    unittest.main()
