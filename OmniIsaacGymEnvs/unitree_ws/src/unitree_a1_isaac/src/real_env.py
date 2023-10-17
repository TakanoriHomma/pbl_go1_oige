#!/usr/bin/env python
from turtle import done
import rospy

import torch

class RealEnv:

    def __init__(self) -> None:
        pass

    def step(self, action):
        print(action)

        state = torch.ones(48, device="cuda:0")
                # obs = torch.cat((base_lin_vel,
                #                  base_ang_vel,
                #                  projected_gravity,
                #                  commands_scaled,
                #                  dof_pos_scaled,
                #                  dof_vel*dof_vel_scale,
                #                  action
                #                  ))
        
        reward = None   # Gomi
        done = None     # Gomi
        _ = None        # Gomi
    

        return state, reward, done, _