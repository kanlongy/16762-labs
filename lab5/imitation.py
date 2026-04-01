import pickle
import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, TensorDataset


class ILPolicy(nn.Module):
    def __init__(self, obs_dim, action_dim):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(obs_dim, 64),
            nn.ReLU(),
            nn.Linear(64, 64),
            nn.ReLU(),
            nn.Linear(64, action_dim),
        )

    def forward(self, obs):
        return self.net(obs)


def train(demos_path='demos.pkl', save_path='imitation_policy.pt',
          lr=0.001, epochs=500, batch_size=256):
    with open(demos_path, 'rb') as f:
        data = pickle.load(f)

    X = torch.tensor(data['X'], dtype=torch.float32)
    y = torch.tensor(data['y'], dtype=torch.float32)
    print(f'Loaded {len(X)} transitions  obs_dim={X.shape[1]}  action_dim={y.shape[1]}')

    dataset = TensorDataset(X, y)
    loader = DataLoader(dataset, batch_size=batch_size, shuffle=True)

    obs_dim = X.shape[1]
    action_dim = y.shape[1]
    policy = ILPolicy(obs_dim, action_dim)

    optimizer = torch.optim.Adam(policy.parameters(), lr=lr)
    loss_fn = nn.MSELoss()

    for epoch in range(1, epochs + 1):
        policy.train()
        total_loss = 0.0
        for obs_batch, action_batch in loader:
            pred = policy(obs_batch)
            loss = loss_fn(pred, action_batch)
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            total_loss += loss.item() * len(obs_batch)
        if epoch % 50 == 0 or epoch == 1:
            print(f'Epoch {epoch:4d}/{epochs}  loss={total_loss / len(X):.6f}')

    torch.save(policy, save_path)
    print(f'Saved imitation policy to {save_path}')


if __name__ == '__main__':
    train()
