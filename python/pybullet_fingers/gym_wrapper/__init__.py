from gym.envs.registration import register

register(id='finger-v0',
    entry_point='pybullet_fingers.gym_wrapper.envs:Finger',
)
