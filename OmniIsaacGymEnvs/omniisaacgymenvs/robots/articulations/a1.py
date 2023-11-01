# Copyright (c) 2018-2022, NVIDIA Corporation
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from typing import Optional
import numpy as np
import torch
from omni.isaac.core.prims import RigidPrimView
# from omni.isaac.quadruped.robots import Unitree
from omni.isaac.core.robots.robot import Robot
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omniisaacgymenvs.utils.usd_utils.create_instanceable_assets import convert_asset_instanceable


import numpy as np
import torch

from pxr import PhysxSchema

class A1(Robot):
    def __init__(
        self,
        prim_path: str,
        name: Optional[str] = "A1",
        usd_path: Optional[str] = None,
        translation: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None,
    ) -> None:
        """[summary]
        """
        
        
        self._usd_path = usd_path
        self._name = name

        if self._usd_path is None:
            assets_root_path = get_assets_root_path()
            if assets_root_path is None:
                carb.log_error("Could not find nucleus server with /Isaac folder")
            # self._usd_path = assets_root_path + "/Isaac/Robots/Unitree/a1.usd"
            self._usd_path = "/isaac-sim/OmniIsaacGymEnvs/omniisaacgymenvs/robots/articulations/a1_instanceable.usd"
            # self._usd_path = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/2022.2.0/Isaac/Samples/Orbit/Robots/Unitree/A1/a1_instanceable.usd"
        add_reference_to_stage(self._usd_path, prim_path)
        
        
        

        super().__init__(
            prim_path=prim_path,
            name=name,
            position=translation,
            orientation=orientation,
            # articulation_controller=None,
        )

        self._dof_names = ["RR_hip",
                           "RL_hip",
                           "FL_hip",
                           "FR_hip",
                           "FL_thigh",
                           "FR_thigh",
                           "RL_thigh",
                           "RR_thigh",
                           "FL_calf",
                           "RL_calf",
                           "FR_calf",
                           "RR_calf"]
        # self._dof_names = ["FL_hip",
        #                     "FR_hip",
        #                     "RL_hip",
        #                     "RR_hip",
        #                     "FL_thigh",
        #                     "FR_thigh",
        #                     "RL_thigh",
        #                     "RR_thigh",
        #                     "FL_calf",
        #                     "FR_calf",
        #                     "RL_calf",
        #                     "RR_calf"]
        



    @property
    def dof_names(self):
        return self._dof_names

    def set_a1_properties(self, stage, prim):
        for link_prim in prim.GetChildren():
            if link_prim.HasAPI(PhysxSchema.PhysxRigidBodyAPI): 
                rb = PhysxSchema.PhysxRigidBodyAPI.Get(stage, link_prim.GetPrimPath())
                rb.GetDisableGravityAttr().Set(False)
                rb.GetRetainAccelerationsAttr().Set(False)
                rb.GetLinearDampingAttr().Set(0.0)
                rb.GetMaxLinearVelocityAttr().Set(1000.0)
                rb.GetAngularDampingAttr().Set(0.0)
                rb.GetMaxAngularVelocityAttr().Set(64/np.pi*180)

    def prepare_contacts(self, stage, prim):
        for link_prim in prim.GetChildren():
            if link_prim.HasAPI(PhysxSchema.PhysxRigidBodyAPI): 
                if "_hip" not in str(link_prim.GetPrimPath()):
                    rb = PhysxSchema.PhysxRigidBodyAPI.Get(stage, link_prim.GetPrimPath())
                    rb.CreateSleepThresholdAttr().Set(0)
                    cr_api = PhysxSchema.PhysxContactReportAPI.Apply(link_prim)
                    cr_api.CreateThresholdAttr().Set(0)
                    
    
