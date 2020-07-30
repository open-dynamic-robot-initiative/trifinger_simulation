#!/usr/bin/env python3
import unittest

from trifinger_simulation import TriFingerPlatform


class TestTriFingerPlatform(unittest.TestCase):
    def test_timestamps(self):
        platform = TriFingerPlatform(visualization=False, enable_cameras=True)
        action = platform.Action()

        # compute camera update step interval based on configured rates
        camera_update_step_interval = (
            1 / platform.camera_rate_fps
        ) / platform._time_step
        # robot time step in milliseconds
        time_step_ms = platform._time_step * 1000

        # First time step
        t = platform.append_desired_action(action)
        first_stamp_ms = platform.get_timestamp_ms(t)
        first_stamp_s = first_stamp_ms / 1000

        object_pose = platform.get_object_pose(t)
        camera_obs = platform.get_camera_observation(t)
        self.assertEqual(first_stamp_ms, object_pose.timestamp)
        self.assertEqual(first_stamp_ms, camera_obs.cameras[0].timestamp)
        self.assertEqual(first_stamp_ms, camera_obs.cameras[1].timestamp)
        self.assertEqual(first_stamp_ms, camera_obs.cameras[2].timestamp)

        # Test time stamps of observations t+1
        object_pose_next = platform.get_object_pose(t + 1)
        camera_obs_next = platform.get_camera_observation(t + 1)
        next_stamp_ms = first_stamp_ms + time_step_ms
        self.assertEqual(next_stamp_ms, object_pose_next.timestamp)
        self.assertEqual(next_stamp_ms, camera_obs_next.cameras[0].timestamp)
        self.assertEqual(next_stamp_ms, camera_obs_next.cameras[1].timestamp)
        self.assertEqual(next_stamp_ms, camera_obs_next.cameras[2].timestamp)

        # XXX ===============================================================
        # The following part of the test is disabled as currently everything is
        # updated in each step (i.e. no lower update rate for camera and
        # object).
        return

        # Second time step
        t = platform.append_desired_action(action)
        second_stamp_ms = platform.get_timestamp_ms(t)
        self.assertEqual(second_stamp_ms, first_stamp_ms + time_step_ms)

        # there should not be a new camera observation yet
        object_pose = platform.get_object_pose(t)
        camera_obs = platform.get_camera_observation(t)
        self.assertEqual(first_stamp_s, object_pose.timestamp)
        self.assertEqual(first_stamp_s, camera_obs.cameras[0].timestamp)
        self.assertEqual(first_stamp_s, camera_obs.cameras[1].timestamp)
        self.assertEqual(first_stamp_s, camera_obs.cameras[2].timestamp)

        # do several steps until a new camera/object update is expected
        for _ in range(int(camera_update_step_interval)):
            t = platform.append_desired_action(action)

        nth_stamp_ms = platform.get_timestamp_ms(t)
        nth_stamp_s = nth_stamp_ms / 1000
        self.assertGreater(nth_stamp_ms, second_stamp_ms)

        object_pose = platform.get_object_pose(t)
        camera_obs = platform.get_camera_observation(t)
        self.assertEqual(nth_stamp_s, object_pose.timestamp)
        self.assertEqual(nth_stamp_s, camera_obs.cameras[0].timestamp)
        self.assertEqual(nth_stamp_s, camera_obs.cameras[1].timestamp)
        self.assertEqual(nth_stamp_s, camera_obs.cameras[2].timestamp)


if __name__ == "__main__":
    import rosunit

    rosunit.unitrun(
        "trifinger_simulation", "test_trifinger_platform", TestTriFingerPlatform,
    )
