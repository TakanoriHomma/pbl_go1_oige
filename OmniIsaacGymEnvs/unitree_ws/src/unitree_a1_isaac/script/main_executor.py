#!/usr/bin/env python
import rospy
import torch
import numpy as np
from scipy.spatial.transform import Rotation as R
from MPC import MPC
from qre_msgs.msg import LowGymCmd
from unitree_legged_msgs.msg import LowState

class ROSPETS(object):
    def __init__(self):
        rospy.init_node('ros_pets')

        # declear var
        self.policy = MPC()
        self.low_cmd = LowGymCmd()
        self.low_state = LowState()

        # publisher
        self.command_pub = rospy.Publisher('low_gym_cmd', LowGymCmd, queue_size=1)

        # subscirber
        rospy.Subscriber('low_state', LowState, self.ros_get_obs)

        self.rate = rospy.Rate(5)

        counter = 0
        while not rospy.is_shutdown():
            obs = self.parser_observation()
            if obs is None:
                continue
            obs[0:10] = [0.3375,0.0133,0.05211,0.0030,0.0346,0.0041,-0.00384,-0.006175,0.0365,0.00467]
            acts = self.policy.act(obs)
            self.parser_action(acts, obs)
            self.ros_do_act(self.low_cmd)

            self.rate.sleep()
            counter += 1
    
    def parser_action(self, acts, obs):
        # parse action to executable 12 dim actionfg
        assert len(acts) == 12, 'Not match action size'
        actions = []
        for i in range(len(acts)):
            curr_state = obs[10 + i]
            action = curr_state + acts[i]
            actions.append(action)
        _action = []
        _action = actions
        # _action[0:3], _action[3:6] = actions[3:6], actions[0:3]
        # _action[6:9], _action[9:12] = actions[9:12], actions[6:9]
        self.low_cmd.q = _action

    def parser_observation(self):
        # parse ob
        # (body_z, body_roll, body_pitch, body_yaw, body_vel_x, body_vel_y, body_vel_z, body_ang_roll, 
        #   body_ang_pitch, body_ang_yaw) + joint_angle
        body_z = 0.32
        flag = False
        for i in range(len(self.low_state.imu.quaternion)):
            if self.low_state.imu.quaternion[i] != 0.0:
                flag = True
        if not flag:
            return None
        roll, pitch, yaw = R.from_quat(self.low_state.imu.quaternion).as_euler('xyz')
        vel_x = self.low_state.imu.accelerometer[0] * 0.1
        vel_y = self.low_state.imu.accelerometer[1] * 0.1
        vel_z = self.low_state.imu.accelerometer[2] * 0.1
        ang_vel_roll = self.low_state.imu.gyroscope[0]
        ang_vel_pitch = self.low_state.imu.gyroscope[1]
        ang_vel_yaw = self.low_state.imu.gyroscope[2] 
        joint_angle = []
        for m in self.low_state.motorState[:12]:
            joint_angle.append(m.q)
        obs = np.array([body_z, roll, pitch, yaw, vel_x, vel_y, vel_z, ang_vel_roll,
            ang_vel_pitch, ang_vel_yaw])
        obs = np.concatenate((obs, joint_angle))
        return obs

    def ros_get_obs(self, msg):
        self.low_state.levelFlag = msg.levelFlag
        self.low_state.commVersion = msg.commVersion
        self.low_state.robotID = msg.robotID
        self.low_state.SN = msg.SN
        self.low_state.bandWidth = msg.bandWidth
        self.low_state.imu = msg.imu
        self.low_state.motorState = msg.motorState
        self.low_state.footForce = msg.footForce
        self.low_state.footForceEst = msg.footForceEst
        self.low_state.tick = msg.tick
        self.low_state.wirelessRemote = msg.wirelessRemote
        self.low_state.reserve = msg.reserve
        self.low_state.crc = msg.crc

    def ros_do_act(self, acts):
        self.command_pub.publish(acts)

if __name__ == "__main__":
    ROSPETS()
