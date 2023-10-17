from __future__ import division
from __future__ import print_function
from __future__ import absolute_import

import numpy as np
import gym

import torch
from torch import nn as nn
from torch.nn import functional as F

from DotmapUtils import get_required_argument
from config.utils import swish, get_affine_params
from env.A1 import Env


TORCH_DEVICE = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')


class PtModel(nn.Module):

    def __init__(self, ensemble_size, in_features, out_features):
        super().__init__()

        self.num_nets = ensemble_size

        self.in_features = in_features
        self.out_features = out_features

        self.lin0_w, self.lin0_b = get_affine_params(ensemble_size, in_features, 200)

        self.lin1_w, self.lin1_b = get_affine_params(ensemble_size, 200, 200)

        self.lin2_w, self.lin2_b = get_affine_params(ensemble_size, 200, 200)

        self.lin3_w, self.lin3_b = get_affine_params(ensemble_size, 200, 200)

        self.lin4_w, self.lin4_b = get_affine_params(ensemble_size, 200, out_features)

        self.inputs_mu = nn.Parameter(torch.zeros(in_features), requires_grad=False)
        self.inputs_sigma = nn.Parameter(torch.zeros(in_features), requires_grad=False)

        self.max_logvar = nn.Parameter(torch.ones(1, out_features // 2, dtype=torch.float32) / 2.0)
        self.min_logvar = nn.Parameter(- torch.ones(1, out_features // 2, dtype=torch.float32) * 10.0)

    def compute_decays(self):

        lin0_decays = 0.000025 * (self.lin0_w ** 2).sum() / 2.0
        lin1_decays = 0.00005 * (self.lin1_w ** 2).sum() / 2.0
        lin2_decays = 0.000075 * (self.lin2_w ** 2).sum() / 2.0
        lin3_decays = 0.000075 * (self.lin3_w ** 2).sum() / 2.0
        lin4_decays = 0.0001 * (self.lin4_w ** 2).sum() / 2.0

        return lin0_decays + lin1_decays + lin2_decays + lin3_decays + lin4_decays

    def fit_input_stats(self, data):

        mu = np.mean(data, axis=0, keepdims=True)
        sigma = np.std(data, axis=0, keepdims=True)
        sigma[sigma < 1e-12] = 1.0
        self.inputs_mu.data = torch.from_numpy(mu).to(TORCH_DEVICE).float()
        self.inputs_sigma.data = torch.from_numpy(sigma).to(TORCH_DEVICE).float()

    def forward(self, inputs, ret_logvar=False):

        # Transform inputs
        inputs = (inputs - self.inputs_mu) / self.inputs_sigma

        inputs = inputs.matmul(self.lin0_w) + self.lin0_b
        inputs = swish(inputs)

        inputs = inputs.matmul(self.lin1_w) + self.lin1_b
        inputs = swish(inputs)

        inputs = inputs.matmul(self.lin2_w) + self.lin2_b
        inputs = swish(inputs)

        inputs = inputs.matmul(self.lin3_w) + self.lin3_b
        inputs = swish(inputs)

        inputs = inputs.matmul(self.lin4_w) + self.lin4_b

        mean = inputs[:, :, :self.out_features // 2]

        logvar = inputs[:, :, self.out_features // 2:]
        logvar = self.max_logvar - F.softplus(self.max_logvar - logvar)
        logvar = self.min_logvar + F.softplus(logvar - self.min_logvar)

        if ret_logvar:
            return mean, logvar

        return mean, torch.exp(logvar)


class A1ConfigModule:
    ENV_NAME = "A1"
    TASK_HORIZON = 1000
    NTRAIN_ITERS = 300
    NROLLOUTS_PER_ITER = 1
    PLAN_HOR = 20
    MODEL_IN, MODEL_OUT = 34, 22 #MODEL_IN=action+observation MODEL_OUT=next_observation
    GP_NINDUCING_POINTS = 300

    def __init__(self):
        self.ENV = Env()
        self.NN_TRAIN_CFG = {"epochs": 5}
        self.OPT_CFG = {
            "Random": {
                "popsize": 2500
            },
            "CEM": {
                "popsize": 200,
                "num_elites": 30,
                "max_iters": 5,
                "alpha": 0.1
            }
        }

    # @staticmethod
    # def obs_preproc(obs):
    #     if isinstance(obs, np.ndarray):
    #         # return np.concatenate([obs[:, 1:2], np.sin(obs[:, 2:3]), np.cos(obs[:, 2:3]), obs[:, 3:]], axis=1)
    #         return obsobservations
    #     elif isinstance(obs, torch.Tensor):
    #         print('######obs is torch.tensor########')
    #         return torch.cat([
    #             obs[:, 1:2],
    #             obs[:, 2:3].sin(),
    #             obs[:, 2:3].cos(),
    #             obs[:, 3:]
    #         ], dim=1)

    @staticmethod
    def obs_preproc(obs):
        return obs

    # @staticmethod
    # def obs_postproc(obs, pred):

    #     assert isinstance(obs, torch.Tensor)

    #     return torch.cat([
    #         pred[:, :1],
    #         obs[:, 1:] + pred[:, 1:]
    #     ], dim=1)

    @staticmethod
    def obs_postproc(obs, pred):

        assert isinstance(obs, torch.Tensor)

        return pred

    # @staticmethod
    # def targ_proc(obs, next_obs):

    #     if isinstance(obs, np.ndarray):
    #         # return np.concatenate([next_obs[:, :1], next_obs[:, 1:] - obs[:, 1:]], axis=1)
    #         return np.concatenate([next_obs[:1], next_obs[1:] - obs[1:]])
    #     elif isinstance(obs, torch.Tensor):
    #         return torch.cat([
    #             next_obs[:, :1],
    #             next_obs[:, 1:] - obs[:, 1:]
    #         ], dim=1)

    @staticmethod
    def targ_proc(next_obs):
        return next_obs

    @staticmethod
    def obs_cost_fn(obs):
        #obs_col = ['body_x','body_y','body_z','body_roll','body_pitch','body_yaw','body_vx','body_vy','body_vz','body_wx','body_wy','body_wz',
        # 'FR_hip_angle','FR_thigh_angle','FR_calf_angle','FL_hip_angle','FL_thigh_angle','FL_calf_angle',
        # 'RR_hip_angle','RR_thigh_angle','RR_calf_angle','RL_hip_angle','RL_thigh_angle','RL_calf_angle',
        # 'FR_hip_vel','FR_thigh_vel','FR_calf_vel','FL_hip_vel','FL_thigh_vel','FL_calf_vel','RR_hip_vel',
        # 'RR_thigh_vel','RR_calf_vel','RL_hip_vel','RL_thigh_vel','RL_calf_vel']
        return torch.where(obs[:,4]>0.2,-2*obs[:,4],0) + torch.where(torch.abs(obs[:,3])>np.pi/6,torch.abs(obs[:,3]),0)\
                + torch.where(obs[:,0]<0.25,obs[:,0]*2,0)\
                + torch.where(torch.abs(obs[:,2]+np.pi/36)>np.pi/9,torch.abs(obs[:,2]+np.pi/36)*2.0,0) + torch.where(torch.abs(obs[:,1])>np.pi/9,torch.abs(obs[:,1]),0)
                # + torch.where(torch.abs(obs[:,7])>0.2,torch.abs(obs[:,7]),0) + torch.where(torch.abs(obs[:,8])>0.2,torch.abs(obs[:,8])*2.0,0)
                # + torch.where(torch.abs(obs[:,2]+np.pi/36)>np.pi/9,torch.abs(obs[:,2]+np.pi/36)*2.0,0) + torch.where(torch.abs(obs[:,1])>np.pi/9,torch.abs(obs[:,1]),0)\

    @staticmethod
    def ac_cost_fn(acs):
        return 0 * (acs ** 2).sum(dim=1)
        # cost = -5*torch.var(acs,0)
        # return cost[:,1]+cost[:,2]+cost[:,4]+cost[:,5]+cost[:,7]+cost[:,8]+cost[:,10]+cost[:,11]

    def nn_constructor(self, model_init_cfg):

        ensemble_size = get_required_argument(model_init_cfg, "num_nets", "Must provide ensemble size")

        # load_model = model_init_cfg.get("load_model", False)

        # assert load_model is False, 'Has yet to support loading model'

        model = PtModel(ensemble_size,
                        self.MODEL_IN, self.MODEL_OUT * 2).to(TORCH_DEVICE)
        # # * 2 because we output both the mean and the variance

        model.optim = torch.optim.Adam(model.parameters(), lr=0.001)

        dict_ = torch.load('model/model_weight.pth')
        dict_['inputs_mu'],dict_['inputs_sigma'] = dict_['inputs_mu'].reshape(34),dict_['inputs_sigma'].reshape(34)
        # print(dict_['inputs_mu'])
        load_model = True
        if load_model:
            model.load_state_dict(dict_)

        return model


CONFIG_MODULE = A1ConfigModule
