#!/usr/bin/env python
import threading
import time
import rospy
import ros_numpy
import numpy as np
from sensor_msgs.msg import Image
from unitree_legged_msgs.msg import LowState, LowCmd
from a1_utilities.a1_sensor_process import *

class ROSExecutor:
  def __init__(
    self,
    policy,
    use_high_command = False,
    control_freq = 50, frame_interval = 4, Kp = 40, Kd = 0.4
  ):
    rospy.init_node('ros_locotransformer')
    self.policy = policy
    self.continue_thread = False
    self.control_freq = control_freq
    self.frame_extract = frame_interval

    self.Kp = Kp
    self.Kd = Kd

    self.execution_thread = None

    self.use_high_command = use_high_command
    self.ros_command = LowCmd()
    self.observation = LowState()
  
    # image observations
    self.depth = Image()
    self.rgb = Image()

    # publisher
    self.low_cmd_pub = rospy.Publisher('/low_gym_cmd', LowCmd, queue_size=1)

    # subscriber
    self._low_state_sub = rospy.Subscriber('/low_state', LowState, self.low_state_callback)
    self._dep_sub = rospy.Subscriber('/depth_array', Image, self.depth_callback)
    self._rgb_sub = rospy.Subscriber('/rgb_array', Image, self.rgb_callback)

  def low_state_callback(self, msg):
    self.observation.levelFlag = msg.levelFlag
    self.observation.commVersion = msg.commVersion
    self.observation.robotID = msg.robotID
    self.observation.SN = msg.SN
    self.observation.bandWidth = msg.bandWidth
    self.observation.imu = msg.imu
    self.observation.motorState = msg.motorState
    self.observation.footForce = msg.footForce
    self.observation.footForceEst = msg.footForceEst
    self.observation.tick = msg.tick
    self.observation.wirelessRemote = msg.wirelessRemote
    self.observation.reserve = msg.reserve
    self.observation.crc = msg.crc

  def rgb_callback(self, msg):
    self.rgb = ros_numpy.numpify(msg)

  def depth_callback(self, msg):
    self.depth = ros_numpy.numpify(msg)

  def parse_command(self, command):
    self.ros_command.q = command[0::5]
    self.ros_command.Kp = command[1::5]
    self.ros_command.dq = command[2::5]
    self.ros_command.Kd = command[3::5]
    self.ros_command.tau = command[4::5]

  def warmup_observations(self):
    self.last_action = np.zeros(12)

    # read one frame
    self.depth_scale, self.curr_frame = 0.001, self.depth
    for i in range(self.frame_extract*3+1):
        # fill action
        action = self.policy.get_action(
          self.observation, self.curr_frame, self.depth_scale,
          self.last_action
        )

        self.last_action = action 
        time.sleep(0.05)
    print("Policy thread initialization done!")

  def main_execution(self):
    count = 0
    if hasattr(self.policy.pf, "cuda_cxt"):
      self.policy.pf.cuda_cxt.push()

    while self.continue_thread:
      start_time = time.time()
      # Get observation
      robot_observation = self.observation
      # Get frame every time
      depth_scale, curr_frame = 0.001, self.depth

      # compute next action
      curr_frame = (curr_frame - np.mean(curr_frame)) / np.sqrt(np.var(curr_frame))
      action = self.policy.get_action(
        robot_observation, curr_frame, depth_scale,
        self.last_action
      )
      self.last_action = action

      # prepare command
      if self.use_high_command:
        command = prepare_high_level_cmd(action)
      else:
        command = prepare_position_cmd(action, self.Kp, self.Kd)

      self.parse_command(command)

      # publish LowCmd
      self.low_cmd_pub.publish(self.ros_command)

      end_time = time.time()
      # control loop frequency
      count += 1

      delay = end_time - start_time
      delay_time = 1 / self.control_freq - delay
      time.sleep(max(0, delay_time))

    if hasattr(self.policy.pf, "cuda_cxt"):
      self.policy.pf.cuda_cxt.pop()

  def start_thread(self):
    print("Start policy thread called")
    self.continue_thread = True
    self.execution_thread = threading.Thread(target=self.main_execution)
    self.execution_thread.start()

  def stop_thread(self):
    print("Stop policy thread called")
    self.continue_thread = False
    self.execution_thread.join()

  def execute(self, execution_time):
    time.sleep(1)
    self.warmup_observations()
    self.start_thread()
    time.sleep(execution_time) # RUN POLICY FOR TEN SECONDS?
    self.stop_thread()
    self.policy.write()
