cmake_minimum_required(VERSION 3.0.2)
project(qre_msgs)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  message_generation
  std_msgs
)

add_message_files(
  FILES
  BodyPose.msg
  JointCMD.msg
  Point.msg
  PointArray.msg
  Joints.msg
  Contacts.msg
  ContactsStamped.msg
  Pose.msg
  LowGymCmd.msg
  HighGymCmd.msg
  )

# Generate services in the 'srv' folder
add_service_files(
  FILES
  SetBodyPose.srv
  SetControl.srv
)

# Generate actions in the 'action' folder
# add_action_files(
#  FILES
#  grasp.action
#  GoToPosition.action  
#)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS
    actionlib_msgs
    message_runtime
    std_msgs
#  DEPENDS system_lib
)
