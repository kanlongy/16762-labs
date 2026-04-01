import os
import torch
import numpy as np
import gymnasium as gym
from rl import TouchEnv
from tianshou.env import DummyVectorEnv
from tianshou.data import Collector, CollectStats

log_dir = 'log'
all_folders = [os.path.join(log_dir, f) for f in os.listdir(log_dir) if f.startswith('PPOExperiment')]
latest_folder = max(all_folders, key=os.path.getmtime)
subfolders = [f for f in os.listdir(latest_folder) if os.path.isdir(os.path.join(latest_folder, f))]
seed_folder = subfolders[0]

checkpoint_path = os.path.join(latest_folder, seed_folder, 'policy.pt')
print(checkpoint_path)

env = gym.make('TouchEnv', render_mode='human')
venv = DummyVectorEnv([lambda: env])

data = torch.load(checkpoint_path, weights_only=False)
policy = data.policy if hasattr(data, 'policy') else data
test_collector = Collector[CollectStats](policy=policy, env=venv)
policy.eval()
result = test_collector.collect(n_episode=1, render=0.0, reset_before_collect=True)
print(f"Final episode reward: {result.returns.mean()}, length: {result.lens.mean()}")

