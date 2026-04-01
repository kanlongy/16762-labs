import os
import torch
import pickle
import numpy as np
import gymnasium as gym
from rl import TouchEnv
from tianshou.data import Batch
from tianshou.env import DummyVectorEnv

def collect_demos(n_demos=1000):
    log_dir = 'log'
    all_folders = [os.path.join(log_dir, f) for f in os.listdir(log_dir) if f.startswith('PPOExperiment')]
    latest_folder = max(all_folders, key=os.path.getmtime)
    subfolders = [f for f in os.listdir(latest_folder) if os.path.isdir(os.path.join(latest_folder, f))]
    seed_folder = subfolders[0]

    checkpoint_path = os.path.join(latest_folder, seed_folder, 'policy.pt')
    print(checkpoint_path)

    # Setup Env
    env = gym.make('TouchEnv')
    venv = DummyVectorEnv([lambda: env])

    # Load Policy
    data = torch.load(checkpoint_path, weights_only=False)
    policy = data.policy if hasattr(data, 'policy') else data
    policy.eval()

    X = []
    y = []
    for i in range(n_demos):
        obs, info = venv.reset(seed=np.random.randint(1000000))

        terminated = False
        truncated = False
        while not terminated and not truncated:
            with torch.no_grad():
                result = policy(Batch(obs=obs, info=info))
                action = result.act

            # TODO: ------------- start --------------
            # Append obs,action to X,y; step venv forward
            X.append(obs[0])
            y.append(action[0])
            obs, reward, terminated, truncated, info = venv.step(action)
            terminated = terminated[0]
            truncated = truncated[0]
            # TODO: -------------- end ---------------

    # TODO: ------------- start --------------
    # Save all demos to .pkl file
    with open('demos.pkl', 'wb') as f:
        pickle.dump({'X': np.array(X), 'y': np.array(y)}, f)
    print(f'Saved {len(X)} transitions to demos.pkl')
    # TODO: -------------- end ---------------

if __name__ == '__main__':
    collect_demos(n_demos=1000)
