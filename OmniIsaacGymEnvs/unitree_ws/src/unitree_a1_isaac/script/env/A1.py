from __future__ import division
from __future__ import print_function
from __future__ import absolute_import

import os

import numpy as np
from gym import utils
from env.pybulletenv import pybulletenv 


class Env(pybulletenv):
    def __init__(self):
        super().__init__()
        self.prev_qpos = None
        self.observation_shape = 22
        self.action_shape = 12
        self.action_high = 0.1*np.ones(12)
        self.action_low = -0.1*np.ones(12)
        self.frame_skip = 25

    def step(self, action, obs):
        # self.prev_qpos = np.copy(self.model.data.qpos.flat)
        self.do_simulation(action,obs,self.frame_skip)
        obs,done = self._get_obs()

        # reward_ctrl = -0.1 * np.square(action).sum()
        # reward_run = ob[0] - 0.0 * np.square(ob[2])
        # reward = reward_run + reward_ctrl

        reward = 1
        return obs, reward, done, {}

    # def _get_obs(self):
    #     # return np.concatenate([
    #         # (self.model.data.qpos.flat[:1] - self.prev_qpos[:1]) / self.dt,
    #         # self.model.data.qpos.flat[1:],
    #         # self.model.data.qvel.flat,
    #         # np.array([1,1,1,1,1,1,1,1,1,1,1,1]),
    #         # np.array([1,1,1,1,1,1,1,1,1,1,1,1]),
    #     # ])
    #     return np.array([1,1,1,1,1,1,1,1,1,1,1,1])

    def reset_model(self):
        # qpos = self.init_qpos + np.random.normal(loc=0, scale=0.001, size=self.model.nq)
        # qvel = self.init_qvel + np.random.normal(loc=0, scale=0.001, size=self.model.nv)
        # self.set_state(qpos, qvel)
        # self.prev_qpos = np.copy(self.model.data.qpos.flat)
        return self._get_obs()

    # def viewer_setup(self):
    #     self.viewer.cam.distance = self.model.stat.extent * 0.25
    #     self.viewer.cam.elevation = -55
