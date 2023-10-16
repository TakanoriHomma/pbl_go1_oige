#!/usr/bin/env python3
import threading
import time
import sys
import os
import rospy
import ros_numpy
from a1_utilities.a1_sensor_process import *
from a1_utilities.predefined_pose import move_to_sit, move_to_stand
from sensor_msgs.msg import Image
from unitree_legged_msgs.msg import LowState, LowCmd


class ROSExecutor:
  def __init__(
    self,
    robot_controller,
    realsense_device,
    use_high_command=False,
    control_freq = 50, frame_interval = 4, Kp = 40, Kd = 0.4
  ):
    rospy.init_node('ros_executor')
    self.robot_controller = robot_controller
    self.realsense_device = realsense_device
    self.use_high_command = use_high_command
    self.control_freq = control_freq
    self.frame_extract = frame_interval
    self.Kp = Kp
    self.Kd = Kd

    self.continue_thread = False
    self.execution_thread = None
    self.gym_cmd = None

    self.ros_robot_observation = LowState()

    # publisher
    self.low_state_pub = rospy.Publisher('/low_state', LowState, queue_size=1)
    self.depth_pub = rospy.Publisher('/depth_array', Image, queue_size=1)
    self.rgb_pub = rospy.Publisher('/rgb_array', Image, queue_size=1)

    # subscriber
    self._low_cmd_sub = rospy.Subscriber('/low_gym_cmd', LowCmd, self.cmd_callback)

  def cmd_callback(self, msg):
    self.gym_cmd = msg.q

  def parse_robot_observation(self, observation):
    self.ros_robot_observation.levelFlag = observation.levelFlag
    self.ros_robot_observation.commVersion = observation.commVersion
    self.ros_robot_observation.robotID = observation.robotID
    self.ros_robot_observation.SN = observation.SN
    self.ros_robot_observation.bandWidth = observation.bandWidth
    self.ros_robot_observation.imu = observation.imu
    self.ros_robot_observation.motorState = observation.motorState
    self.ros_robot_observation.footForce = observation.footForce
    self.ros_robot_observation.footForceEst = observation.footForceEst
    self.ros_robot_observation.tick = observation.tick
    self.ros_robot_observation.wirelessRemote = observation.wirelessRemote
    self.ros_robot_observation.reserve = observation.reserve
    self.ros_robot_observation.crc = observation.crc
  
  def warmup_observations(self):
    self.last_action = np.zeros(12)

    # read one frame
    for i in range(self.frame_extract*3+1):
        action = [0.1, 0.8, -1.5, 
                  -0.1, 0.8, -1.5, 
                  0.1, 1.0, -1.5, 
                  -0.1, 1.0, -1.5]

        self.last_action = action 
        time.sleep(0.05)
    print("Policy thread initialization done!")

  def main_execution(self):
    count = 0

    while self.continue_thread:
      start_time = time.time()

      # Get robot observation
      robot_observation = self.robot_controller.get_observation()

      # Parse LowState
      self.parse_robot_observation(robot_observation)

      if self.realsense_device is not None:
        # Get depth observation
        _, depth_frame = self.realsense_device.get_depth_frame()
        _, rgb_frame = self.realsense_device.get_rgb_frame()

        # Convert images to ros data
        depth = ros_numpy.msgify(Image, depth_frame, encoding='mono16')
        rgb = ros_numpy.msgify(Image, rgb_frame, encoding='bgr8')

        # publish image
        self.depth_pub.publish(depth)
        self.rgb_pub.publish(rgb)

      # publish state observations
      self.low_state_pub.publish(self.ros_robot_observation)

      # compute next action
      action = self.gym_cmd
      if action is None:
          action = self.last_action
      self.last_action = action

      # prepare command
      if self.use_high_command:
        command = prepare_high_level_cmd(action)
      else:
        command = prepare_position_cmd(action, self.Kp, self.Kd)
      self.robot_controller.set_action(command)

      end_time = time.time()
      count += 1

      delay = end_time - start_time
      delay_time = 1 / self.control_freq - delay
      time.sleep(max(0, delay_time))

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
    if self.realsense_device is not None:
      self.realsense_device.start_thread()
      self.robot_controller.start_thread()

      time.sleep(1)

      # Warmup to start
      self.warmup_observations()

      # Get robot to standing position
      move_to_stand(self.robot_controller)

      self.start_thread()
      time.sleep(execution_time) # RUN POLICY FOR TEN SECONDS?
      self.stop_thread()

      # Get robot to sitting position
      move_to_sit(self.robot_controller)

      # Terminate all processes
      self.realsense_device.stop_thread()
      self.robot_controller.stop_thread()
    else:
      self.robot_controller.start_thread()

      time.sleep(1)

      # Warmup to start
      self.warmup_observations()

      # Get robot to standing position
      move_to_stand(self.robot_controller)

      self.start_thread()
      time.sleep(execution_time) # RUN POLICY FOR TEN SECONDS?
      self.stop_thread()

      # Get robot to sitting position
      move_to_sit(self.robot_controller)

      # Terminate all processes
      self.robot_controller.stop_thread()

if __name__ == '__main__':
  ROSExecutor()
