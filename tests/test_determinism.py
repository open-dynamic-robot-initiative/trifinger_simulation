#!/usr/bin/env python3
import unittest
import numpy as np

from trifinger_simulation.sim_finger import SimFinger


class TestSimulationDeterminisim(unittest.TestCase):
    """This test verifies that the simulation always behaves deterministically.

    When starting from the same position and sending the same commands, the
    result should be the same.
    """

    def test_constant_torque(self):
        """Compare two runs sending a constant torque.

        In each run the finger is reset to a fixed position and a constant
        torque is applied for a number of steps.  The final observation of each
        run is compared.  If the simulation behaves deterministically, the
        observations should be equal.
        """
        finger = SimFinger(
            finger_type="fingerone",
        )

        start_position = [0.5, -0.7, -1.5]
        action = finger.Action(torque=[0.3, 0.3, 0.3])

        def run():
            finger.reset_finger_positions_and_velocities(start_position)
            for _ in range(30):
                finger._set_desired_action(action)
                finger._step_simulation()
            return finger._get_latest_observation()

        first_run = run()
        second_run = run()

        np.testing.assert_array_equal(first_run.torque, second_run.torque)
        np.testing.assert_array_equal(first_run.position, second_run.position)
        np.testing.assert_array_equal(first_run.velocity, second_run.velocity)


if __name__ == "__main__":
    unittest.main()
