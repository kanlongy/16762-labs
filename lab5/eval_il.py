import os
import torch
import numpy as np
import gymnasium as gym
from rl import TouchEnv
from imitation import ILPolicy
# Setup Env
env = gym.make('TouchEnv', render_mode='human')

# Load Policy
policy = torch.load('imitation_policy.pt', weights_only=False)
policy.eval()

obs, info = env.reset(seed=np.random.randint(9000000))

terminated = False
truncated = False
while not terminated and not truncated:
    with torch.no_grad():
        action = policy(torch.Tensor(obs))

    obs, reward, terminated, truncated, info = env.step(action)
    # print(reward)

