#!/usr/bin/env python3
from collections import namedtuple
import pytest
import numpy as np

from trifinger_simulation import TriFingerPlatform


def test_timestamps_no_camera_delay():
    platform = TriFingerPlatform(
        visualization=False, enable_cameras=True, camera_delay_steps=0
    )
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
    assert first_stamp_s == camera_obs.cameras[0].timestamp
    assert first_stamp_s == camera_obs.cameras[1].timestamp
    assert first_stamp_s == camera_obs.cameras[2].timestamp

    # Test time stamps of observations t+1 (with the current implementation,
    # the observation should be exactly the same as for t).
    camera_obs_next = platform.get_camera_observation(t + 1)
    assert first_stamp_s == camera_obs_next.cameras[0].timestamp
    assert first_stamp_s == camera_obs_next.cameras[1].timestamp
    assert first_stamp_s == camera_obs_next.cameras[2].timestamp

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


def test_camera_timestamps_with_camera_delay_simple():
    # Basic test with simple values: Camera update every 2 steps, delay of 1
    # step.
    platform = TriFingerPlatform(
        visualization=False, enable_cameras=True, camera_delay_steps=1
    )
    action = platform.Action()

    # set camera rate to 100 fps so we need less stepping in this test
    platform.camera_rate_fps = 500
    assert platform._compute_camera_update_step_interval() == 2

    initial_stamp_s = platform.get_timestamp_ms(0) / 1000

    # first step triggers camera (we get the initial observation at this point)
    t = platform.append_desired_action(action)
    assert t == 0
    camera_obs = platform.get_camera_observation(t)
    assert initial_stamp_s == camera_obs.cameras[0].timestamp, f"t={t}"
    assert initial_stamp_s == camera_obs.cameras[1].timestamp, f"t={t}"
    assert initial_stamp_s == camera_obs.cameras[2].timestamp, f"t={t}"

    trigger1_stamp_s = platform.get_timestamp_ms(t) / 1000

    # in second step observation is ready but still has stamp zero
    t = platform.append_desired_action(action)
    camera_obs = platform.get_camera_observation(t)
    assert trigger1_stamp_s == camera_obs.cameras[0].timestamp, f"t={t}"
    assert trigger1_stamp_s == camera_obs.cameras[1].timestamp, f"t={t}"
    assert trigger1_stamp_s == camera_obs.cameras[2].timestamp, f"t={t}"

    # in third step new observation is triggered but due to delay we still get
    # the old one
    t = platform.append_desired_action(action)
    camera_obs = platform.get_camera_observation(t)
    assert trigger1_stamp_s == camera_obs.cameras[0].timestamp, f"t={t}"
    assert trigger1_stamp_s == camera_obs.cameras[1].timestamp, f"t={t}"
    assert trigger1_stamp_s == camera_obs.cameras[2].timestamp, f"t={t}"

    trigger2_stamp_s = platform.get_timestamp_ms(t) / 1000
    assert trigger2_stamp_s > trigger1_stamp_s

    # in forth step the new observation is ready
    t = platform.append_desired_action(action)
    camera_obs = platform.get_camera_observation(t)
    assert trigger2_stamp_s == camera_obs.cameras[0].timestamp, f"t={t}"
    assert trigger2_stamp_s == camera_obs.cameras[1].timestamp, f"t={t}"
    assert trigger2_stamp_s == camera_obs.cameras[2].timestamp, f"t={t}"

    # again trigger but we get previous observation due to delay
    t = platform.append_desired_action(action)
    camera_obs = platform.get_camera_observation(t)
    assert trigger2_stamp_s == camera_obs.cameras[0].timestamp, f"t={t}"
    assert trigger2_stamp_s == camera_obs.cameras[1].timestamp, f"t={t}"
    assert trigger2_stamp_s == camera_obs.cameras[2].timestamp, f"t={t}"

    trigger3_stamp_s = platform.get_timestamp_ms(t) / 1000
    assert trigger3_stamp_s > trigger2_stamp_s

    # and there should be an update again
    t = platform.append_desired_action(action)
    camera_obs = platform.get_camera_observation(t)
    assert trigger3_stamp_s == camera_obs.cameras[0].timestamp, f"t={t}"
    assert trigger3_stamp_s == camera_obs.cameras[1].timestamp, f"t={t}"
    assert trigger3_stamp_s == camera_obs.cameras[2].timestamp, f"t={t}"


def test_camera_timestamps_with_camera_delay_less_than_rate():
    # A bit more complex example (probably redundant with the simple one above
    # but since I already implemented it, let's keep it).

    platform = TriFingerPlatform(
        visualization=False, enable_cameras=True, camera_delay_steps=10
    )
    action = platform.Action()

    # ensure the camera frame rate is set to 10 Hz
    assert platform.camera_rate_fps == 10
    assert platform._compute_camera_update_step_interval() == 100

    first_stamp_s = platform.get_timestamp_ms(0) / 1000

    # The start is a bit tricky because of the initial observation which has
    # timestamp 0 but the next observation is triggered in the first step,
    # resulting in the same timestamp.  So first step 100 times, until the next
    # camera update is triggered.
    for i in range(100):
        t = platform.append_desired_action(action)

        # In each step, we still should see the camera observation from step 0
        cameras = platform.get_camera_observation(t).cameras
        assert first_stamp_s == cameras[0].timestamp, f"i={i}, t={t}"
        assert first_stamp_s == cameras[1].timestamp, f"i={i}, t={t}"
        assert first_stamp_s == cameras[2].timestamp, f"i={i}, t={t}"

    assert t == 99

    # one more step to trigger the next camera update
    t = platform.append_desired_action(action)
    second_stamp_s = platform.get_timestamp_ms(t) / 1000

    # The next observation should be triggered now but due to the delay, we
    # should still see the old observation for the next 9 steps
    for i in range(9):
        t = platform.append_desired_action(action)
        cameras = platform.get_camera_observation(t).cameras
        assert first_stamp_s == cameras[0].timestamp, f"i={i}, t={t}"
        assert first_stamp_s == cameras[1].timestamp, f"i={i}, t={t}"
        assert first_stamp_s == cameras[2].timestamp, f"i={i}, t={t}"

    # after the next step, we should see an updated camera observation which
    # again persists for 100 steps
    for i in range(100):
        t = platform.append_desired_action(action)
        cameras = platform.get_camera_observation(t).cameras
        assert second_stamp_s == cameras[0].timestamp, f"i={i}, t={t}"
        assert second_stamp_s == cameras[1].timestamp, f"i={i}, t={t}"
        assert second_stamp_s == cameras[2].timestamp, f"i={i}, t={t}"

    # and now the next update should be there
    t = platform.append_desired_action(action)
    camera_obs = platform.get_camera_observation(t)
    assert second_stamp_s < camera_obs.cameras[0].timestamp


def test_camera_timestamps_with_camera_delay_more_than_rate():
    # In this test the delay is higher than the camera rate, so this results in
    # the effective rate to be reduced.
    # Use small numbers (camera update interval 2 and delay 3) to keep the test
    # manageable.

    platform = TriFingerPlatform(
        visualization=False, enable_cameras=True, camera_delay_steps=3
    )
    action = platform.Action()

    # set high camera rate so we need less stepping in this test
    platform.camera_rate_fps = 500
    assert platform._compute_camera_update_step_interval() == 2

    initial_stamp_s = platform.get_timestamp_ms(0) / 1000

    # first step triggers camera (we get the initial observation at this point)
    t = platform.append_desired_action(action)
    assert t == 0
    camera_obs = platform.get_camera_observation(t)
    assert initial_stamp_s == camera_obs.cameras[0].timestamp, f"t={t}"
    assert initial_stamp_s == camera_obs.cameras[1].timestamp, f"t={t}"
    assert initial_stamp_s == camera_obs.cameras[2].timestamp, f"t={t}"

    trigger1_stamp_s = platform.get_timestamp_ms(t) / 1000

    # now it takes 3 steps until we actually see the new observation
    t = platform.append_desired_action(action)
    t = platform.append_desired_action(action)
    camera_obs = platform.get_camera_observation(t)
    assert initial_stamp_s == camera_obs.cameras[0].timestamp, f"t={t}"
    assert initial_stamp_s == camera_obs.cameras[1].timestamp, f"t={t}"
    assert initial_stamp_s == camera_obs.cameras[2].timestamp, f"t={t}"
    t = platform.append_desired_action(action)
    camera_obs = platform.get_camera_observation(t)
    assert trigger1_stamp_s == camera_obs.cameras[0].timestamp, f"t={t}"
    assert trigger1_stamp_s == camera_obs.cameras[1].timestamp, f"t={t}"
    assert trigger1_stamp_s == camera_obs.cameras[2].timestamp, f"t={t}"

    # Only now, one step later, the next update is triggered
    t = platform.append_desired_action(action)
    camera_obs = platform.get_camera_observation(t)
    assert trigger1_stamp_s == camera_obs.cameras[0].timestamp, f"t={t}"
    assert trigger1_stamp_s == camera_obs.cameras[1].timestamp, f"t={t}"
    assert trigger1_stamp_s == camera_obs.cameras[2].timestamp, f"t={t}"

    trigger2_stamp_s = platform.get_timestamp_ms(t) / 1000
    assert trigger2_stamp_s > trigger1_stamp_s

    # And again it takes 3 steps until we actually see the new observation
    t = platform.append_desired_action(action)
    t = platform.append_desired_action(action)
    camera_obs = platform.get_camera_observation(t)
    assert trigger1_stamp_s == camera_obs.cameras[0].timestamp, f"t={t}"
    assert trigger1_stamp_s == camera_obs.cameras[1].timestamp, f"t={t}"
    assert trigger1_stamp_s == camera_obs.cameras[2].timestamp, f"t={t}"
    t = platform.append_desired_action(action)
    camera_obs = platform.get_camera_observation(t)
    assert trigger2_stamp_s == camera_obs.cameras[0].timestamp, f"t={t}"
    assert trigger2_stamp_s == camera_obs.cameras[1].timestamp, f"t={t}"
    assert trigger2_stamp_s == camera_obs.cameras[2].timestamp, f"t={t}"


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
