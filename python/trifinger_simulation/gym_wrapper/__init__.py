from gym.envs.registration import register
from trifinger_simulation.gym_wrapper.envs import trifinger_cube_env

register(
    id="TriFingerReach-v0",
    entry_point="trifinger_simulation.gym_wrapper.envs.trifinger_reach:TriFingerReach",
)
register(
    id="TriFingerPush-v0",
    entry_point="trifinger_simulation.gym_wrapper.envs.trifinger_push:TriFingerPush",
)

for difficulty in [1, 2, 3, 4]:
    initializer = trifinger_cube_env.RandomInitializer(difficulty=difficulty)
    kwargs= {'initializer': initializer}
    register(
        id="TriFingerCubeDifficulty{}-v1".format(difficulty),
        entry_point="trifinger_simulation.gym_wrapper.envs.trifinger_cube_env:TriFingerCubeEnv",
        kwargs=kwargs,
    )

