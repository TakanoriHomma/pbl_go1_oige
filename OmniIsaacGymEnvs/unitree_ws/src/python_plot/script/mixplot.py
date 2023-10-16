#!/usr/bin/env python3
import rosbag
import numpy as np
import csv
import matplotlib.pyplot as plt
import sys
import os

args = sys.argv
print(len(args))
assert len(args)>=2, "you must specify the argument."

# get path
filename=os.path.normpath(os.path.join(os.getcwd(),args[1]))
print(filename)

# read from bag file
bag = rosbag.Bag(filename)
np_poses=None
for topic, msg, t in bag.read_messages():
    if topic=="/low_state_msg":
        np_pose=np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])
        np_pose[0,0]=msg.motorState[2].q
        np_pose[0,1]=msg.motorState[2].dq
        np_pose[0,2]=msg.motorState[2].ddq
        np_pose[0,3]=msg.motorState[5].q
        np_pose[0,4]=msg.motorState[5].dq
        np_pose[0,5]=msg.motorState[5].ddq
        np_pose[0,6]=msg.motorState[8].q
        np_pose[0,7]=msg.motorState[8].dq
        np_pose[0,8]=msg.motorState[8].ddq
        np_pose[0,9]=msg.motorState[11].q
        np_pose[0,10]=msg.motorState[11].dq
        np_pose[0,11]=msg.motorState[11].ddq
        np_pose[0,12]=msg.tick
        # np_pose[0,4]=t.motorState[0].nsecs
        if np_poses is None:
            np_poses=np_pose
        else:
            np_poses=np.append(np_poses,np_pose,axis=0)

def read_csv(filename):
    data = []
    with open(filename, 'r') as f:
        reader = csv.reader(f)
        for row in reader:
            data.append(list(map(float, row)))
    return data

# CSVファイルを読み込む
pos_data = read_csv('pos_data.csv')
vel_data = read_csv('vel_data.csv')

# データをtransposeする（列を行に変換する）
pos_data = list(map(list, zip(*pos_data)))
vel_data = list(map(list, zip(*vel_data)))

sim_t = [i/30 for i in range(len(pos_data[0]))]

# reform time
start_sec=np_poses[0,12]
# start_nsec=np_poses[0,4]
t=np.zeros(np_poses.shape[0],dtype='float32')
for i in range(np_poses.shape[0]):
    t[i]=(np_poses[i,12]-start_sec)/500 #+(np_poses[i,4]-start_nsec)/1000000000.0

# plot    
plt.subplot(2,2,1)
# plt.title("time vs x,y")
plt.title("FR_thigh")
plt.plot(t, np_poses[:,0], 'c', label="real_q")
plt.plot(t, np_poses[:,1], 'b', label="real_dq")
plt.plot(sim_t, pos_data[0], 'm', label='sim_q')
plt.plot(sim_t, vel_data[0], 'r', label='sim_dq')
plt.xlabel("time[s]")
plt.ylabel('q[rad], dq[rad/s]') 
plt.legend()
plt.xlim([-0, 30])  # x軸の範囲を0から最大時間までに設定します
plt.ylim([-2.5, 1.5]) 

plt.subplot(2,2,2)
plt.title("FL_thigh")
plt.plot(t, np_poses[:,3], 'c', label="real_q")
plt.plot(t, np_poses[:,4], 'b', label="real_dq")
plt.plot(sim_t, pos_data[1], 'm', label='sim_q')
plt.plot(sim_t, vel_data[1], 'r', label='sim_dq')
plt.xlabel("time[s]")
plt.ylabel('q[rad], dq[rad/s]') 
plt.legend()
plt.xlim([-0, 30])  # x軸の範囲を0から最大時間までに設定します
plt.ylim([-2.5, 1.5]) 

plt.subplot(2,2,3)
plt.title("RR_thigh")
plt.plot(t, np_poses[:,6], 'c', label="real_q")
plt.plot(t, np_poses[:,7], 'b', label="real_dq")
plt.plot(sim_t, pos_data[3], 'm', label='sim_q')
plt.plot(sim_t, vel_data[3], 'r', label='sim_dq')
plt.xlabel("time[s]")
plt.ylabel('q[rad], dq[rad/s]') 
plt.legend()
plt.xlim([-0, 30])  # x軸の範囲を0から最大時間までに設定します
plt.ylim([-2.5, 1.5]) 

plt.subplot(2,2,4)
plt.title("RL_thigh")
plt.plot(t, np_poses[:,9], 'c', label="real_q")
plt.plot(t, np_poses[:,10], 'b', label="real_dq")
plt.plot(sim_t, pos_data[2], 'm', label='sim_q')
plt.plot(sim_t, vel_data[2], 'r', label='sim_dq')
plt.xlabel("time[s]")
plt.ylabel('q[rad], dq[rad/s]') 
plt.legend()
plt.xlim([-0, 30])  # x軸の範囲を0から最大時間までに設定します
plt.ylim([-2.5, 1.5]) 

# plt.subplot(122)
# plt.title("x vs y")
# plt.plot(np_poses[:,0], np_poses[:,1], 'g')
plt.show()

bag.close()
