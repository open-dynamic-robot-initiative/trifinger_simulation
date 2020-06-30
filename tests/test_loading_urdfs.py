import unittest
import os

import pybullet
from pybullet_fingers.base_finger import BaseFinger
from pybullet_fingers.sim_finger import SimFinger


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
        finger_type_data = BaseFinger.finger_type_data
        for key in finger_type_data.keys():
            try:
                SimFinger(
                    time_step=0.004,
                    enable_visualization=False,
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
        finger_type_data = BaseFinger.finger_type_data
        for key in finger_type_data.keys():
            try:
                os.environ["ROS_PACKAGE_PATH"] = " "
                SimFinger(
                    time_step=0.004,
                    enable_visualization=False,
                    finger_type=key,
                )
            except pybullet.error as e:
                self.fail(
                    "Failed to import the local copies of the urdf for"
                    "SimFinger(finger_type={}): {}".format(key, e)
                )


if __name__ == "__main__":
    unittest.main()
