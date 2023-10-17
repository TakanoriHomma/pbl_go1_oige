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

from unitree_legged_msgs.msg import LowCmd, LowState, MotorCmd, MotorState

import math


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
            "observation_space": spaces.Box(np.ones(42) * -np.Inf, np.ones(42) * np.Inf),
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



start_up = True




class UnitreePublisher:
    def __init__(self, robot_name):
        self.robot_name = robot_name
        self.FRhip_pub = rospy.Publisher("/" + self.robot_name + "_gazebo/FR_hip_controller/command", MotorCmd, queue_size=1)
        self.FRthigh_pub = rospy.Publisher("/" + self.robot_name + "_gazebo/FR_thigh_controller/command", MotorCmd, queue_size=1)
        self.FRcalf_pub = rospy.Publisher("/" + self.robot_name + "_gazebo/FR_calf_controller/command", MotorCmd, queue_size=1)
        self.FLhip_pub = rospy.Publisher("/" + self.robot_name + "_gazebo/FL_hip_controller/command", MotorCmd, queue_size=1)
        self.FLthigh_pub = rospy.Publisher("/" + self.robot_name + "_gazebo/FL_thigh_controller/command", MotorCmd, queue_size=1)
        self.FLcalf_pub = rospy.Publisher("/" + self.robot_name + "_gazebo/FL_calf_controller/command", MotorCmd, queue_size=1)
        self.RRhip_pub = rospy.Publisher("/" + self.robot_name + "_gazebo/RR_hip_controller/command", MotorCmd, queue_size=1)
        self.RRthigh_pub = rospy.Publisher("/" + self.robot_name + "_gazebo/RR_thigh_controller/command", MotorCmd, queue_size=1)
        self.RRcalf_pub = rospy.Publisher("/" + self.robot_name + "_gazebo/RR_calf_controller/command", MotorCmd, queue_size=1)
        self.RLhip_pub = rospy.Publisher("/" + self.robot_name + "_gazebo/RL_hip_controller/command", MotorCmd, queue_size=1)
        self.RLthigh_pub = rospy.Publisher("/" + self.robot_name + "_gazebo/RL_thigh_controller/command", MotorCmd, queue_size=1)
        self.RLcalf_pub = rospy.Publisher("/" + self.robot_name + "_gazebo/RL_calf_controller/command", MotorCmd, queue_size=1)



    def publish_commands(self, FRhip, FRthigh, FRcalf, FLhip, FLthigh, FLcalf, RRhip, RRthigh, RRcalf, RLhip, RLthigh, RLcalf):
        self.FRhip_pub.publish(FRhip)
        self.FRthigh_pub.publish(FRthigh)
        self.FRcalf_pub.publish(FRcalf)
        self.FLhip_pub.publish(FLhip)
        self.FLthigh_pub.publish(FLthigh)
        self.FLcalf_pub.publish(FLcalf)
        self.RRhip_pub.publish(RRhip)
        self.RRthigh_pub.publish(RRthigh)
        self.RRcalf_pub.publish(RRcalf)
        self.RLhip_pub.publish(RLhip)
        self.RLthigh_pub.publish(RLthigh)
        self.RLcalf_pub.publish(RLcalf)




class GymCommands():
    def __init__(self):
        self.agent = PpoPlayerContinuous(params)
        self.agent.restore("../checkpoints/A1_bipedal.pth")
        self.state = None
        self.last_action = [0] * 12

        rospy.init_node('unitree_gazebo_servo')

        self.default_dof_pos = [ 
                    0.0, 0.0, 0.0, 0.0,
                    2.3, 2.3, 2.1, 2.1,
                    -2.3, -2.3, -1.5, -1.5]        

        robot_name = "a1"

        self.gym_cmd = LowState()
        imu_sub = rospy.Subscriber("/trunk_imu", MotorState, self.imuCallback, queue_size=1)
        self.FRhip_sub = rospy.Subscriber("/" + robot_name + "_gazebo/FR_hip_controller/state", MotorState, self.FRhipCallback, queue_size=1)
        self.FRthigh_sub = rospy.Subscriber("/" + robot_name + "_gazebo/FR_thigh_controller/state", MotorState, self.FRthighCallback, queue_size=1)
        self.FRcalf_sub = rospy.Subscriber("/" + robot_name + "_gazebo/FR_calf_controller/state", MotorState, self.FRcalfCallback, queue_size=1)
        self.FLhip_sub = rospy.Subscriber("/" + robot_name + "_gazebo/FL_hip_controller/state", MotorState, self.FLhipCallback, queue_size=1)
        self.FLthigh_sub = rospy.Subscriber("/" + robot_name + "_gazebo/FL_thigh_controller/state", MotorState, self.FLthighCallback, queue_size=1)
        self.FLcalf_sub = rospy.Subscriber("/" + robot_name + "_gazebo/FL_calf_controller/state", MotorState, self.FLcalfCallback, queue_size=1)
        self.RRhip_sub = rospy.Subscriber("/" + robot_name + "_gazebo/RR_hip_controller/state", MotorState, self.RRhipCallback, queue_size=1)
        self.RRthigh_sub = rospy.Subscriber("/" + robot_name + "_gazebo/RR_thigh_controller/state", MotorState, self.RRthighCallback, queue_size=1)
        self.RRcalf_sub = rospy.Subscriber("/" + robot_name + "_gazebo/RR_calf_controller/state", MotorState, self.RRcalfCallback, queue_size=1)
        self.RLhip_sub = rospy.Subscriber("/" + robot_name + "_gazebo/RL_hip_controller/state", MotorState, self.RLhipCallback, queue_size=1)
        self.RLthigh_sub = rospy.Subscriber("/" + robot_name + "_gazebo/RL_thigh_controller/state", MotorState, self.RLthighCallback, queue_size=1)
        self.RLcalf_sub = rospy.Subscriber("/" + robot_name + "_gazebo/RL_calf_controller/state", MotorState, self.RLcalfCallback, queue_size=1)

        self.state_callback(self.gym_cmd)

        

        unitree_publisher = UnitreePublisher(robot_name)

        counter = 0



        while not rospy.is_shutdown():
            if self.state is None: 
                continue

            state = torch.tensor(self.state, dtype=torch.float).cpu()
            _action = self.agent.get_action(state, is_determenistic=True)
            _action = _action.to('cpu').detach().numpy().copy()

            

            _q = 0.5 * _action + self.default_dof_pos

            q = [0] * 12
            q[0] = _q[2]
            q[1] = _q[6]
            q[2] = _q[10]
            q[3] = _q[1]
            q[4] = _q[5]
            q[5] = _q[9]
            q[6] = _q[3]
            q[7] = _q[7]
            q[8] = _q[11]
            q[9] = _q[0]
            q[10] = _q[4]
            q[11] = _q[8]




            FRhip = MotorCmd()
            FRthigh =  MotorCmd()
            FRcalf =  MotorCmd()
            FLhip =  MotorCmd()
            FLthigh =  MotorCmd()
            FLcalf =  MotorCmd()
            RRhip =  MotorCmd()
            RRthigh =  MotorCmd()
            RRcalf =  MotorCmd()
            RLhip =  MotorCmd()
            RLthigh =  MotorCmd()
            RLcalf =  MotorCmd()

            FRhip.mode = 10
            FRthigh.mode = 10
            FRcalf.mode = 10
            FLhip.mode = 10
            FLthigh.mode = 10
            FLcalf.mode = 10
            RRhip.mode = 10
            RRthigh.mode = 10
            RRcalf.mode = 10
            RLhip.mode = 10
            RLthigh.mode = 10
            RLcalf.mode = 10
        
            if counter < 500:
                
                FRhip.q = 0.0
                FRthigh.q = 2.3
                FRcalf.q = -2.3
                FLhip.q = 0.0
                FLthigh.q = 2.3
                FLcalf.q = -2.3
                RRhip.q = 0.0
                RRthigh.q = 2.1
                RRcalf.q = -1.5
                RLhip.q = 0.0
                RLthigh.q = 2.1
                RLcalf.q = -1.5   

                counter += 1

            else:


                # FRhip.q = q[1]
                # FRthigh.q = q[5]
                # FRcalf.q = q[9]
                # FLhip.q = q[0]
                # FLthigh.q = q[4]
                # FLcalf.q = q[8]
                # RRhip.q = q[3]
                # RRthigh.q = q[7]
                # RRcalf.q = q[11]
                # RLhip.q = q[2]
                # RLthigh.q = q[6]
                # RLcalf.q = q[10] 

                FRhip.q = q[0]
                FRthigh.q = q[1]
                FRcalf.q = q[2]
                FLhip.q = q[3]
                FLthigh.q = q[4]
                FLcalf.q = q[5]
                RRhip.q = q[6]
                RRthigh.q = q[7]
                RRcalf.q = q[8]
                RLhip.q = q[9]
                RLthigh.q = q[10]
                RLcalf.q = q[11] 

            FRhip.Kp = 45
            FRthigh.Kp = 45
            FRcalf.Kp = 45
            FLhip.Kp = 45
            FLthigh.Kp = 45
            FLcalf.Kp = 45
            RRhip.Kp = 45
            RRthigh.Kp = 45
            RRcalf.Kp = 45
            RLhip.Kp = 45
            RLthigh.Kp = 45
            RLcalf.Kp = 45   


            FRhip.Kd = 3
            FRthigh.Kd = 3
            FRcalf.Kd = 3
            FLhip.Kd = 3
            FLthigh.Kd = 3
            FLcalf.Kd = 3
            RRhip.Kd = 3
            RRthigh.Kd = 3
            RRcalf.Kd = 3
            RLhip.Kd = 3
            RLthigh.Kd = 3
            RLcalf.Kd = 3


            self.last_action = _action

            unitree_publisher.publish_commands(FRhip, FRthigh, FRcalf, FLhip, FLthigh, FLcalf, RRhip, RRthigh, RRcalf, RLhip, RLthigh, RLcalf)

    def imuCallback(self,msg):
        self.gym_cmd.imu.gyroscope[0] = msg.angular_velocity.x
        self.gym_cmd.imu.gyroscope[1] = msg.angular_velocity.y
        self.gym_cmd.imu.gyroscope[2] = msg.angular_velocity.z
        



    def FRhipCallback(self,msg):
        self.gym_cmd.motorState[0].q = msg.q
        self.gym_cmd.motorState[0].dq = msg.dq

    def FRthighCallback(self,msg):
        self.gym_cmd.motorState[1].q = msg.q
        self.gym_cmd.motorState[1].dq = msg.dq

    def FRcalfCallback(self,msg):
        self.gym_cmd.motorState[2].q = msg.q
        self.gym_cmd.motorState[2].dq = msg.dq

    def FLhipCallback(self,msg):
        self.gym_cmd.motorState[3].q = msg.q
        self.gym_cmd.motorState[3].dq = msg.dq

    def FLthighCallback(self,msg):
        self.gym_cmd.motorState[4].q = msg.q
        self.gym_cmd.motorState[4].dq = msg.dq

    def FLcalfCallback(self,msg):
        self.gym_cmd.motorState[5].q = msg.q
        self.gym_cmd.motorState[5].dq = msg.dq

    def RRhipCallback(self,msg):
        self.gym_cmd.motorState[6].q = msg.q
        self.gym_cmd.motorState[6].dq = msg.dq

    def RRthighCallback(self,msg):
        self.gym_cmd.motorState[7].q = msg.q
        self.gym_cmd.motorState[7].dq = msg.dq

    def RRcalfCallback(self,msg):
        self.gym_cmd.motorState[8].q = msg.q
        self.gym_cmd.motorState[8].dq = msg.dq

    def RLhipCallback(self,msg):
        self.gym_cmd.motorState[9].q = msg.q
        self.gym_cmd.motorState[9].dq = msg.dq

    def RLthighCallback(self,msg):
        self.gym_cmd.motorState[10].q = msg.q
        self.gym_cmd.motorState[10].dq = msg.dq

    def RLcalfCallback(self,msg):
        self.gym_cmd.motorState[11].q = msg.q
        self.gym_cmd.motorState[11].dq = msg.dq


    def state_callback(self, msg):

        base_ang_vel = []
        for m in msg.imu.gyroscope:
            base_ang_vel.append(0.25 * m)

        _dof_pos_scaled = []
        for i, m in enumerate(msg.motorState[:12]):
            _dof_pos_scaled.append(1.0 * (m.q - self.default_dof_pos[i]))
        dof_pos_scaled = [0] * 12

        dof_pos_scaled[0] = _dof_pos_scaled[2]
        dof_pos_scaled[1] = _dof_pos_scaled[6]
        dof_pos_scaled[2] = _dof_pos_scaled[10]
        dof_pos_scaled[3] = _dof_pos_scaled[1]
        dof_pos_scaled[4] = _dof_pos_scaled[5]
        dof_pos_scaled[5] = _dof_pos_scaled[9]
        dof_pos_scaled[6] = _dof_pos_scaled[3]
        dof_pos_scaled[7] = _dof_pos_scaled[7]
        dof_pos_scaled[8] = _dof_pos_scaled[11]
        dof_pos_scaled[9] = _dof_pos_scaled[0]
        dof_pos_scaled[10] = _dof_pos_scaled[4]
        dof_pos_scaled[11] = _dof_pos_scaled[8]


        commands = [1.0, 0.0, 0.0]
        commands_scaled = []
        scales = [2.0, 2.0, 0.25]
        for i, m in enumerate(commands):
            commands_scaled.append(scales[i] * m)

        _dof_vel_scaled = []
        for m in msg.motorState[:12]:
            _dof_vel_scaled.append(0.05 * m.dq)
        dof_vel_scaled = [0] * 12
        dof_vel_scaled[0] = _dof_vel_scaled[2]
        dof_vel_scaled[1] = _dof_vel_scaled[6]
        dof_vel_scaled[2] = _dof_vel_scaled[10]
        dof_vel_scaled[3] = _dof_vel_scaled[1]
        dof_vel_scaled[4] = _dof_vel_scaled[5]
        dof_vel_scaled[5] = _dof_vel_scaled[9]
        dof_vel_scaled[6] = _dof_vel_scaled[3]
        dof_vel_scaled[7] = _dof_vel_scaled[7]
        dof_vel_scaled[8] = _dof_vel_scaled[11]
        dof_vel_scaled[9] = _dof_vel_scaled[0]
        dof_vel_scaled[10] = _dof_vel_scaled[4]
        dof_vel_scaled[11] = _dof_vel_scaled[8]



        self.state = np.concatenate([base_ang_vel, commands_scaled, dof_pos_scaled, dof_vel_scaled, self.last_action])



if __name__ == '__main__':
    GymCommands()

