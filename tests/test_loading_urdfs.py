import unittest
import os

import pybullet
from trifinger_simulation.sim_finger import SimFinger
from trifinger_simulation import finger_types_data


class TestLoadingURDFs(unittest.TestCase):
    """
    This test verifies that all the URDFs corresponding to
    all the valid finger types can be imported successfully.
    """

    def test_loading_urdfs(self):
        """
        Get the keys corresponding to the valid finger types
        from BaseFinger and try importing their corresponding
        URDFs.
        """
        finger_data = finger_types_data.finger_types_data
        for key in finger_data.keys():
            try:
                SimFinger(
                    finger_type=key,
                )

            except pybullet.error as e:
                self.fail(
                    "Failed to create SimFinger(finger_type={}): {}".format(
                        key, e
                    )
                )

    def test_loading_urdfs_locally(self):
        """
        Get the keys corresponding to the valid finger types
        from BaseFinger and try importing their corresponding
        URDFs.
        """
        finger_data = finger_types_data.finger_types_data
        for key in finger_data.keys():
            try:
                os.environ["ROS_PACKAGE_PATH"] = " "
                SimFinger(
                    finger_type=key,
                )
            except pybullet.error as e:
                self.fail(
                    "Failed to import the local copies of the urdf for"
                    "SimFinger(finger_type={}): {}".format(key, e)
                )


if __name__ == "__main__":
    unittest.main()
