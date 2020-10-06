# this is a test script to be executed for training on the real script
# it depends on stable-baselines3
import gym
from stable_baselines3 import SAC
from stable_baselines3.sac import MlpPolicy
print("imports done")
env = gym.make(
    'trifinger_simulation.gym_wrapper:reach-v0',
    control_rate_s=0.02,
    finger_type="trifingerone",
    enable_visualization=False,
    use_real_robot=True,
    finger_config_suffix="0",
    synchronize=False,
    )
print("gym env made")
model = SAC(MlpPolicy, env, verbose=1)
print("model created, to begin learning")
model.learn(total_timesteps=10000, log_interval=4)
model.save("real-finger-fingering")

del model # remove to demonstrate saving and loading

model = SAC.load("real-finger-fingering")

import pdb as ipdb; ipdb.set_trace()

obs = env.reset()
while True:
    action, _states = model.predict(obs)
    obs, reward, done, info = env.step(action)
    env.render()
    if done:
        obs = env.reset()