#!/usr/bin/env python3
from collections import namedtuple
import pytest
import numpy as np

from trifinger_simulation import TriFingerPlatform


def test_timestamps():
    platform = TriFingerPlatform(visualization=False, enable_cameras=True)
    action = platform.Action()

    # ensure the camera frame rate is set to 10 Hz
    assert platform.camera_rate_fps == 10

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

    camera_obs = platform.get_camera_observation(t)
    assert first_stamp_ms == camera_obs.cameras[0].timestamp
    assert first_stamp_ms == camera_obs.cameras[1].timestamp
    assert first_stamp_ms == camera_obs.cameras[2].timestamp

    # Test time stamps of observations t+1 (with the current implementation,
    # the observation should be exactly the same as for t).
    camera_obs_next = platform.get_camera_observation(t + 1)
    assert first_stamp_ms == camera_obs_next.cameras[0].timestamp
    assert first_stamp_ms == camera_obs_next.cameras[1].timestamp
    assert first_stamp_ms == camera_obs_next.cameras[2].timestamp

    # Second time step
    t = platform.append_desired_action(action)
    second_stamp_ms = platform.get_timestamp_ms(t)
    assert second_stamp_ms == first_stamp_ms + time_step_ms

    # there should not be a new camera observation yet
    camera_obs = platform.get_camera_observation(t)
    assert first_stamp_s == camera_obs.cameras[0].timestamp
    assert first_stamp_s == camera_obs.cameras[1].timestamp
    assert first_stamp_s == camera_obs.cameras[2].timestamp

    # do several steps until a new camera/object update is expected
    # (-1 because there is already one action appended above for the
    # "second time step")
    for _ in range(int(camera_update_step_interval - 1)):
        t = platform.append_desired_action(action)

    nth_stamp_ms = platform.get_timestamp_ms(t)
    nth_stamp_s = nth_stamp_ms / 1000
    assert nth_stamp_ms > second_stamp_ms

    camera_obs = platform.get_camera_observation(t)
    assert nth_stamp_s == camera_obs.cameras[0].timestamp
    assert nth_stamp_s == camera_obs.cameras[1].timestamp
    assert nth_stamp_s == camera_obs.cameras[2].timestamp


def test_get_camera_observation_timeindex():
    platform = TriFingerPlatform(enable_cameras=True)

    # negative time index needs to be rejected
    with pytest.raises(ValueError):
        platform.get_camera_observation(-1)

    t = platform.append_desired_action(platform.Action())
    try:
        platform.get_camera_observation(t)
        platform.get_camera_observation(t + 1)
    except Exception:
        pytest.fail()

    with pytest.raises(ValueError):
        platform.get_camera_observation(t + 2)


def test_object_pose_observation():
    Pose = namedtuple("Pose", ["position", "orientation"])
    pose = Pose([0.1, -0.5, 0], [0, 0, 0.2084599, 0.97803091])

    platform = TriFingerPlatform(initial_object_pose=pose)
    t = platform.append_desired_action(platform.Action())
    obs = platform.get_camera_observation(t)

    np.testing.assert_array_equal(obs.object_pose.position, pose.position)
    np.testing.assert_array_equal(
        obs.filtered_object_pose.position, pose.position
    )
    np.testing.assert_array_almost_equal(
        obs.object_pose.orientation, pose.orientation
    )
    np.testing.assert_array_almost_equal(
        obs.filtered_object_pose.orientation, pose.orientation
    )
