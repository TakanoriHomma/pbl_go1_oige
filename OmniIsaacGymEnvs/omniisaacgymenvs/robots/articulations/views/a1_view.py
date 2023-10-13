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

from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core.prims import RigidPrimView
import torch


class A1View(ArticulationView):
    def __init__(
        self,
        prim_paths_expr: str,
        
        name: Optional[str] = "A1View",
        track_contact_forces=True,
        prepare_contact_sensors=True,
        enable_dof_force_sensors: bool = True
    ) -> None:
        """[summary]
        """

        super().__init__(
            prim_paths_expr=prim_paths_expr,
            name=name,
            reset_xform_properties=False,
            enable_dof_force_sensors = enable_dof_force_sensors
        )
        self._knees = RigidPrimView(prim_paths_expr="/World/envs/.*/A1/.*_calf",
            name="knees_view", reset_xform_properties=False, track_contact_forces=track_contact_forces, prepare_contact_sensors=prepare_contact_sensors)
        self._base = RigidPrimView(prim_paths_expr="/World/envs/.*/A1/trunk",
            name="base_view", reset_xform_properties=False, track_contact_forces=track_contact_forces, prepare_contact_sensors=prepare_contact_sensors)
        self._foots = RigidPrimView(prim_paths_expr="/World/envs/.*/A1/.*_foot",
            name="foot_view", reset_xform_properties=False, track_contact_forces=track_contact_forces, prepare_contact_sensors=prepare_contact_sensors)
        self._foots_heights = self._foots.get_world_poses()
        
        base_pos, _ = self.get_world_poses()
        self.basic=base_pos[:, 1]

    def get_knee_transforms(self):
        return self._knees.get_world_poses()

    def is_knee_below_threshold(self, threshold, ground_heights=None):
        knee_pos, _ = self._knees.get_world_poses()
        knee_heights = knee_pos.view((-1, 4, 3))[:, :, 2]
        if ground_heights is not None:
            knee_heights -= ground_heights
        return (knee_heights[:, 0] < threshold) | (knee_heights[:, 1] < threshold) | (knee_heights[:, 2] < threshold) | (knee_heights[:, 3] < threshold)

    def is_base_below_threshold(self, threshold, ground_heights):
        base_pos, _ = self.get_world_poses()
        base_heights = base_pos[:, 2]
        base_heights -= ground_heights
        self._foots_heights = self._foots.get_world_poses()
        return (base_heights[:] < threshold)
    
    def is_knee_under_line(self, threshold):
        knee_pos, _ = self._knees.get_world_poses()
        knee_heights = knee_pos.view((-1, 4, 3))[:, :, 1]
        # print("standard:",knee_heights)
        knee_heights =torch.abs(torch.abs(knee_heights)- torch.abs(self.basic.view(-1,1)))
        # print("basic",self.basic.view(-1,1))
        # print("after:",knee_heights)
        return (knee_heights[:, 0] > threshold) | (knee_heights[:, 1] > threshold) | (knee_heights[:, 2] > threshold) | (knee_heights[:, 3] > threshold)


    @property
    def get_contact_view(self):
        print(self._physics_view)   
        return self._physics_view.get_force_sensor_forces()

