from gym.envs.registration import register

register(
    id="reach-v0",
    entry_point="trifinger_simulation.gym_wrapper.envs.trifinger_reach:TriFingerReach",
)
register(
    id="push-v0",
    entry_point="trifinger_simulation.gym_wrapper.envs.trifinger_push:TriFingerPush",
)

register(
    id="real_robot_challenge_phase_1-v1",
    entry_point="trifinger_simulation.gym_wrapper.envs.cube_env:CubeEnv",
)

register(
    id="TriFingerReachImages-v1",
    entry_point="trifinger_simulation.gym_wrapper.envs.trifinger_reach_images:TriFingerReachImages",
)
