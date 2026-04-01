import os
import sys
sys.path.insert(0, '/home/ye/mengine')
import numpy as np
import mengine as m
import gymnasium as gym

# pip3 install gymnasium tianshou rliable
# pip3 install -U arch

class TouchEnv(gym.Env):
    def __init__(self, render_mode=None, **kwargs):
        self.env = m.Env(gravity=[0, 0, -1], render=render_mode=='human')
        self.observation_space = gym.spaces.Box(low=-10.0, high=10.0, shape=(3+3+3+5,)) # TODO: Update this based on the dimensionality of your observation vector
        self.action_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(7,))

    def _get_obs(self):
        # TODO: ------------- start --------------
        ee_pos, _ = self.robot.get_link_pos_orient(self.robot.end_effector)
        obj_pos, _ = self.object.get_base_pos_orient()
        ee_pos_local, _ = self.robot.global_to_local_coordinate_frame(ee_pos)
        obj_pos_local, _ = self.robot.global_to_local_coordinate_frame(obj_pos)

        diff=obj_pos_local-ee_pos_local

        joint_angles=self.robot.get_joint_angles(self.robot.controllable_joints)
        lift=joint_angles[2]
        arm_sum = joint_angles[3] + joint_angles[4] + joint_angles[5] + joint_angles[6]
        yaw = joint_angles[7]
        pitch = joint_angles[8]
        roll = joint_angles[9]
        joint_obs = np.array([lift, arm_sum, yaw, pitch, roll])
        obs = np.concatenate([ee_pos_local, obj_pos_local, diff, joint_obs])

        return obs.astype(np.float32)
        # TODO: -------------- end ---------------

    def _get_info(self):
        return {}

    def seed(self, seed):
        np.random.seed(seed)
        # self.np_random.seed(seed)

    def render(self):
        return None

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.seed(seed)

        self.env.reset()
        ground = m.Ground()

        height = np.random.uniform(-0.2, 0.2)

        table = m.URDF(filename=os.path.join(m.directory, 'table', 'table.urdf'), static=True, position=[-1.3, 0, height], orientation=[0, 0, 0, 1])

        self.object = m.Shape(m.Mesh(filename=os.path.join(m.directory, 'ycb', 'mustard.obj'), scale=[1, 1, 1]), static=False, mass=1.0, position=[-0.6-np.random.uniform(0.0, 0.2), np.random.uniform(-0.2, 0.2), 0.85+height], orientation=[0, 0, 0, 1], rgba=None, visual=True, collision=True)

        pos_x = np.random.uniform(-0.1, 0.1)
        pos_y = np.random.uniform(-0.1, 0.1)
        theta = np.random.uniform(-np.pi/4, np.pi/4)
        self.robot = m.Robot.Stretch3(position=[pos_x, pos_y, 0], orientation=[0, 0, theta])
        self.robot.set_joint_angles(angles=[0.9], joints=[4])

        # Let the object settle on the table
        m.step_simulation(steps=10, realtime=False)

        return self._get_obs(), self._get_info()

    def step(self, action):
        # action is delta joint angle change
        scale = 0.025
        action = np.array(action)
        scaled_action = np.concatenate([action[:2]*0.5,         # Base movements
                                        [action[2]*scale],      # Lift joint
                                        [action[3]/4.0*scale]*4,# Arm extension joints
                                        action[4:]*scale,       # Wrist joints
                                        [0, 0]])                # Gripper joint
        current_angles = self.robot.get_joint_angles(self.robot.controllable_joints)
        self.robot.control(current_angles + scaled_action)
        m.step_simulation(steps=10, realtime=self.env.render)

        # TODO: ------------- start --------------
        ee_pos, _ = self.robot.get_link_pos_orient(self.robot.end_effector)
        obj_pos, _ = self.object.get_base_pos_orient()
        dist = np.linalg.norm(np.array(ee_pos) - np.array(obj_pos))
        contact = self.robot.get_contact_points(bodyB=self.object)
        reward = np.exp(-dist) + (10.0 if contact else 0.0) - 0.01
        # TODO: -------------- end ---------------

        observation = self._get_obs()
        info = self._get_info()
        terminated = False
        truncated = False

        return observation, reward, terminated, truncated, info

gym.register(id='TouchEnv', entry_point=TouchEnv, max_episode_steps=75)

from tianshou.highlevel.config import OnPolicyTrainingConfig
from tianshou.highlevel.env import (EnvFactoryRegistered, VectorEnvType)
from tianshou.highlevel.experiment import PPOExperimentBuilder, ExperimentConfig
from tianshou.highlevel.params.algorithm_params import PPOParams
from tianshou.highlevel.trainer import EpochStopCallbackRewardThreshold

def run_experiment():
    experiment = (
        PPOExperimentBuilder(
            EnvFactoryRegistered(
                task='TouchEnv',
                venv_type=VectorEnvType.SUBPROC,
                training_seed=0,
                test_seed=10,
            ),
            ExperimentConfig(
                persistence_enabled=True,
                persistence_base_dir='log',
                watch=True,
                watch_render=1 / 35,
                watch_num_episodes=100,
            ),
            OnPolicyTrainingConfig(
                max_epochs=200,
                epoch_num_steps=2048,
                num_training_envs=8,
                num_test_envs=1,
                test_in_training=False,
                buffer_size=2048,
                collection_step_num_env_steps=2048,
                batch_size=64,
                update_step_num_repetitions=10,
            ),
        )
        .with_ppo_params(
            PPOParams(
                lr=1e-3,
                gamma=0.99,
                gae_lambda=0.95,
                eps_clip=0.2,
                ent_coef=0.01,
                vf_coef=0.5,
                max_grad_norm=None,
                action_bound_method='clip',
                action_scaling=True,
                advantage_normalization=True,
                recompute_advantage=False,
            ),
        )
        .with_actor_factory_default(hidden_sizes=(64, 64))
        .with_critic_factory_default(hidden_sizes=(64, 64))
        .with_epoch_stop_callback(EpochStopCallbackRewardThreshold(195))
        .build()
    )

    experiment.run()

if __name__ == '__main__':
    run_experiment()


