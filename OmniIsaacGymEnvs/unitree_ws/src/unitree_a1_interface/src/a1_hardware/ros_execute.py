#!/usr/bin/env python3
import rospy
import sys

from control_loop_execution.ros_executor import ROSExecutor
from a1_utilities.robot_controller import RobotController
from a1_utilities.realsense import A1RealSense
from a1_utilities.a1_sensor_process import *


if __name__ == "__main__":
  # check python version
  print('python version:', sys.version)  

  execution_time = rospy.get_param('/unitree_a1_interface/execution_time')
  image = rospy.get_param('/unitree_a1_interface/image')
  high_cmd = rospy.get_param('/unitree_a1_interface/high_cmd')
  control_freq = rospy.get_param('/unitree_a1_interface/control_freq')
  frame_interval = rospy.get_param('/unitree_a1_interface/frame_interval')
  Kp = rospy.get_param('/unitree_a1_interface/Kp')
  Kd = rospy.get_param('/unitree_a1_interface/Kd')

  robot_controller = RobotController()
  
  if image:
    realsense = A1RealSense()
    executor = ROSExecutor(
      robot_controller=robot_controller,
      realsense_device=realsense,
      use_high_command=high_cmd,
      control_freq=control_freq,
      frame_interval=frame_interval,
      Kp=Kp,
      Kd=Kd
    )
  else:
    executor = ROSExecutor(
      robot_controller=robot_controller,
      realsense_device=None,
      use_high_command=high_cmd,
      control_freq=control_freq,
      frame_interval=frame_interval,
      Kp=Kp,
      Kd=Kd
    )
  executor.execute(execution_time)
