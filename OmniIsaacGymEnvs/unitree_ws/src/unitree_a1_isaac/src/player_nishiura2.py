#!/usr/bin/env python
from rl_games.common.player import BasePlayer
from rl_games.algos_torch import torch_ext
from rl_games.algos_torch.running_mean_std import RunningMeanStd
from rl_games.common.tr_helpers import unsqueeze_obs
import gym
import torch
from torch import nn
import numpy as np
import time

import rospy
import gym.spaces as spaces
from unitree_legged_msgs.msg import LowState
from unitree_legged_msgs.msg import LowCmd
from sensor_msgs.msg import JointState

NUMBER_OF_JOINTS = 12

def rescale_actions(low, high, action):
    d = (high - low) / 2.0
    m = (high + low) / 2.0
    scaled_action =  action * d + m
    return scaled_action

class PpoPlayerContinuous(BasePlayer):
    def __init__(self, params):
        BasePlayer.__init__(self, params)
        self.network = self.config['network']
        self.actions_num = self.action_space.shape[0]
        self.actions_low = torch.from_numpy(self.action_space.low.copy()).float().cpu()
        self.actions_high = torch.from_numpy(self.action_space.high.copy()).float().cpu()
        self.mask = [False]

        self.normalize_input = self.config['normalize_input']
        self.normalize_value = self.config.get('normalize_value', False)

        obs_shape = self.obs_shape
        config = {
            'actions_num' : self.actions_num,
            'input_shape' : obs_shape,
            'num_seqs' : self.num_agents,
            'value_size': self.env_info.get('value_size',1),
            'normalize_value': self.normalize_value,
            'normalize_input': self.normalize_input,
        } 
        print(obs_shape)
        self.model = self.network.build(config)
        self.model.cpu()
        self.model.eval()
        self.is_rnn = self.model.is_rnn()

    def get_action(self, obs, is_determenistic=False):
        if self.has_batch_dimension == False:
            obs = unsqueeze_obs(obs)
        obs = self._preproc_obs(obs)
        input_dict = {
            'is_train': False,
            'prev_actions': None, 
            'obs' : obs,
            'rnn_states' : self.states
        }
        with torch.no_grad():
            res_dict = self.model(input_dict)
        mu = res_dict['mus']
        action = res_dict['actions']
        self.states = res_dict['rnn_states']
        if is_determenistic:
            current_action = mu
        else:
            current_action = action
        if self.has_batch_dimension == False:
            current_action = torch.squeeze(current_action.detach())

        if self.clip_actions:
            return rescale_actions(self.actions_low, self.actions_high, torch.clamp(current_action, -1.0, 1.0))
        else:
            return current_action

    def restore(self, fn):
        checkpoint = torch.load(fn, map_location=torch.device('cpu'))
        del checkpoint['model']['value_mean_std.running_mean']
        del checkpoint['model']['value_mean_std.running_var']
        del checkpoint['model']['value_mean_std.count']

        self.model.load_state_dict(checkpoint['model'])
        if self.normalize_input and 'running_mean_std' in checkpoint:
            self.model.running_mean_std.load_state_dict(checkpoint['running_mean_std'])


    def reset(self):
        self.init_rnn()

params = {
    "config": {
        "env_name": "real_unitree",
        "env_config": {
            "seed": 1
        },
        "env_info": {
            "agents": 1,
            "action_space": spaces.Box(np.ones(12) * -1., np.ones(12) * 1.),
            "observation_space": spaces.Box(np.ones(36) * -np.Inf, np.ones(36) * np.Inf),
        },
        
        "normalize_input": True,
    },

    "model": {
        "name": "continuous_a2c_logstd",
    },

    "network": {
        "name": "actor_critic",
        "separate": False,

        "space": {
            "continuous": {
                "mu_activation": "None",
                "sigma_activation": "None",
                "mu_init": {
                    "name": "default",
                },
                "sigma_init": {
                    "name": "const_initializer",
                    "val": 0. , # std = 1.
                },
                "fixed_sigma": True,
            },
        },

        "mlp": {
            "units": [256, 128, 64],
            "activation": "elu",
            "d2rl": False,

            "initializer": {
                "name": "default"
            },
            "regularizer": {
                "name": "None"
            },
        },
    },
}

class GymCommands():
    def __init__(self):
        self.agent = PpoPlayerContinuous(params)
        self.agent.restore("../checkpoints/A1.pth")
        self.state = None
        self.gym_cmd = LowCmd()
        self.last_action = [0] * 12

        rospy.init_node("player")
        rospy.Subscriber("a1_gazebo/joint_states", JointState, self.state_callback)
        pub = rospy.Publisher("low_states", LowCmd, queue_size=1)
        rate = rospy.Rate(500)

        self.default_dof_pos = [ 
                    0.1, 0.8, -1.5, 
                   -0.1, 0.8, -1.5, 
                    0.1, 1.0, -1.5, 
                   -0.1, 1.0, -1.5]

        while not rospy.is_shutdown():
            if self.state is None: 
                continue

            state = torch.tensor(self.state, dtype=torch.float).cpu()
            _action = self.agent.get_action(state, is_determenistic=True)
            _action = _action.to('cpu').detach().numpy().copy()

            _q = 0.5 * _action + self.default_dof_pos

            q = [] * 12
            q[0:3], q[3:6] = _q[3:6], _q[0:3]
            q[6:9], q[9:12] = _q[9:12], _q[6:9]

            self.last_action = _action
            for i in range(len(q)):
                self.gym_cmd.motorCmd[i].q = q[i]
                self.gym_cmd.motorCmd[i].Kp = 45
                self.gym_cmd.motorCmd[i].Kd = 3
        
            pub.publish(self.gym_cmd)
            rate.sleep()

    def state_callback(self, msg):

        _dof_pos_scaled = []
        for i, m in enumerate(msg.position[:12]):
            _dof_pos_scaled.append(1.0 * (m - self.default_dof_pos[i]))
        dof_pos_scaled = [] * 12
        dof_pos_scaled[0:3], dof_pos_scaled[3:6] = _dof_pos_scaled[3:6], _dof_pos_scaled[0:3]
        dof_pos_scaled[6:9], dof_pos_scaled[9:12] = _dof_pos_scaled[9:12], _dof_pos_scaled[6:9]

        commands = [1.0, 0.0, 0.0]
        commands_scaled = []
        scales = [2.0, 2.0, 0.25]
        for i, m in enumerate(commands):
            commands_scaled.append(scales[i] * m)

        _dof_vel_scaled = []
        for m in msg.velocity[:12]:
            _dof_vel_scaled.append(0.05 * m)
        dof_vel_scaled = [] * 12
        dof_vel_scaled[0:3], dof_vel_scaled[3:6] = _dof_vel_scaled[3:6], _dof_vel_scaled[0:3]
        dof_vel_scaled[6:9], dof_vel_scaled[9:12] = _dof_vel_scaled[9:12], _dof_vel_scaled[6:9]

        self.state = np.concatenate([dof_pos_scaled, dof_vel_scaled, self.last_action])

if __name__ == "__main__":
    GymCommands()