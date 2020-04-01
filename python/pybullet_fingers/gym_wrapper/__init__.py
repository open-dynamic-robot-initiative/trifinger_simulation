from gym.envs.registration import register

register(
    id='reach-v0',
    entry_point='pybullet_fingers.gym_wrapper.envs.finger_reach:FingerReach',
)
register(
    id='push-v0',
    entry_point='pybullet_fingers.gym_wrapper.envs.finger_push:FingerPush',
)
