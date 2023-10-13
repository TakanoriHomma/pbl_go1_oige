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
import math

from scipy.spatial.transform import Rotation as R

import gym.spaces as spaces
# from qre_msgs.msg import LowCmd

import rospy
from unitree_legged_msgs.msg import LowState, LowCmd
from geometry_msgs.msg import PoseStamped

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
        commands = [1.0, 0.0, 0.0]
        commands_scaled = []
        scales = [2.0, 2.0, 0.25]
        for i, m in enumerate(commands):
            commands_scaled.append(scales[i] * m)
        del checkpoint['model']['value_mean_std.count']
        # del checkpoint['model']['running_mean_std.running_mean']
        # del checkpoint['model']['running_mean_std.running_var']
        # del checkpoint['model']['running_mean_std.count']

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
            "observation_space": spaces.Box(np.ones(38) * -np.Inf, np.ones(38) * np.Inf),
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
        self.agent.restore("/isaac-sim/OmniIsaacGymEnvs/omniisaacgymenvs/runs/Go1/nn/Go1.pth")
        self.state = None
        self.dt = 1/60
        self.gym_cmd = LowCmd()
        self.last_action = [0] * 12

        #IK用
        self.action_rate = 0.1
        self.initial_pose = None
        self.previous_positions = [0] * 12
        rospy.init_node("player")
        rospy.Subscriber("low_state", LowState, self.state_callback)        
        pub = rospy.Publisher("low_gym_cmd", LowCmd, queue_size=1)
        rate = rospy.Rate(30)

        self.default_dof_pos = [ 
                0.0, 0.8, -1.5, 
                0.1, 0.8, -1.5, 
                -0.1, 0.8, -1.5, 
                0.1, 0.8, -1.5]
        
        # self.default_dof_pos = [ 
        #             FRH, FRT, FRC, 
        #             FLH, FLT, FLC, 
        #             RRH, RRT, RRC, 
        #             RLH, RRT, RRC]

        #IsaacGym
        # FLH
        # FRH
        # RLH
        # RRT
        # FLT
        # FRT
        # RLT
        # RRT
        # FLC
        # FRC
        # RLC
        # RRC

        self.gym_dof_pos = [
            0.1, 0.0, 0.1, -0.1,
            0.8, 0.8, 0.8, 0.8,
            -1.5, -1.5, -1.5, -1.5,
        ]
        self.default_Kp = [1.0] * NUMBER_OF_JOINTS
        self.default_Kd = [0.1] * NUMBER_OF_JOINTS
        self.default_tau = [0.0] * NUMBER_OF_JOINTS
        self._q = self.gym_dof_pos

        while not rospy.is_shutdown():
            begin_at = time.time()
            if self.state is None: 
                continue
            
            state = torch.tensor(self.state, dtype=torch.float32).cpu()
            _action = self.agent.get_action(state, is_determenistic=True)
            _action = _action.to('cpu').detach().numpy().copy()

            print(_action)
            self.cmd=LowCmd()
            new_positions = [pos + act * self.dt * self.action_rate  for pos, act in zip(self.previous_positions, _action)]
            for i in range(NUMBER_OF_JOINTS):
                self.cmd.motorCmd[i].q = new_positions[i]
                self.gym_cmd.motorCmd[i].Kp = self.default_Kp[i]
                self.gym_cmd.motorCmd[i].Kd = self.default_Kd[i]
                self.gym_cmd.motorCmd[i].tau = self.default_tau[i]
            self.last_action = _action.tolist()
            pub.publish(self.cmd)
            # print(time.time() - begin_at)
            rate.sleep()

    def state_callback(self, msg):
        euler_angles = self.quaternion_to_euler_xyz(*msg.imu.quaternion)
        state_list = list(euler_angles[:2])  # Using only x and y from the euler angles

        # Extracting MotorState for 12 selected motors
        # Assuming the first 12 motors are relevant; adjust indices if needed
        for motor in msg.motorState[:12]:

            # TODO actionscale
            state_list.append(motor.q)  # Motor position
            state_list.append(motor.dq)  # Motor speed

        # Adding the last action taken
        state_list.extend(self.last_action)
        self.previous_positions = [motor.q for motor in msg.motorState[:12]]
        self.state = state_list


    def quaternion_to_euler_xyz(self,w,x,y,z):
        quaternion = np.stack([x, y, z, w]).T 
        # quaternion = np.stack([0, 0, 0, 1]).T 
        r = R.from_quat(quaternion)
        # euler = r.as_euler('xyz', degrees=True)
        euler = r.as_euler('xyz', degrees=False)
        
        return euler
    
if __name__ == "__main__":
    GymCommands()

