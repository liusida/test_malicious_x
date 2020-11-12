import argparse
import pybullet_envs
import gym
import numpy as np
from pybullet_envs.gym_locomotion_envs import Walker2DBulletEnv, WalkerBaseBulletEnv
from pybullet_envs.robot_locomotors import Walker2D, WalkerBase
from gym.envs.registration import register
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import EvalCallback
from stable_baselines3.common.env_util import make_vec_env


class Walker2D_1(Walker2D):
    def __init__(self):
        # change obs_dim from 22 to 23
        WalkerBase.__init__(self, "walker2d.xml", "torso", action_dim=6, obs_dim=23, power=0.40)
    def calc_state(self):
        # add malicious x
        state = super().calc_state()
        state = np.concatenate([state, [self.body_real_xyz[0]]])
        return state

class Walker2DBulletEnv_1(Walker2DBulletEnv):
    def __init__(self, render=False):
        self.robot = Walker2D_1()
        WalkerBaseBulletEnv.__init__(self, self.robot, render)


def main(args):
    register(id='Walker2DBulletEnv-v1',
         entry_point='test_malicious_x:Walker2DBulletEnv_1',
         max_episode_steps=1000,
         reward_threshold=2500.0)
    env_version = "v1" if args.with_malicious_x else "v0" #v0 is the vanilla Walker2D, v1 is the env with malicious x
    env = make_vec_env(f"Walker2DBulletEnv-{env_version}", seed=args.seed, n_envs=16)
    eval_env = gym.make(f"Walker2DBulletEnv-{env_version}")
    eval_env.seed(args.seed+17)
    eval_callback = EvalCallback(eval_env=eval_env, eval_freq=1000, deterministic=True, render=False)
    model = PPO(policy="MlpPolicy", env=env, tensorboard_log="./tb/", verbose=1)
    model.learn(total_timesteps=3000000, tb_log_name=f"{env_version}_s{args.seed}", callback=eval_callback)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-x", "--with-malicious-x", action="store_true")
    parser.add_argument("-s", "--seed", type=int, default=0)
    args = parser.parse_args()
    main(args)