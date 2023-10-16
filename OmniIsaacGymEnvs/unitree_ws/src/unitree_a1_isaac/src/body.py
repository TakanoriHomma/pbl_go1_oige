# Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
# Use of this source code is governed by the MPL-2.0 license, see LICENSE.

import rospy
from unitree_legged_msgs.msg import LowCmd, LowState, HighState

PosStopF = 2.146e9
VelStopF = 16000.0

class UnitreeModel:
    def __init__(self, robot_name, low_state):
        self.servo_pub = [None] * 12
        self.highState_pub = None
        self.lowCmd = LowCmd()
        self.lowState = LowState()

    def stand(self):
        pass

    def motion_init(self):
        pass

    def send_servo_cmd(self):
        pass

    def move_all_position(self, joint_positions, duration):
        pass
