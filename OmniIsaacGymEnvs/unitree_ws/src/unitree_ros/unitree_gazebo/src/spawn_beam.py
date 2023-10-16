#!/usr/bin/env python

import rospy
import sys
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose

def spawn_box():
    rospy.init_node('spawn_box_node')
    
    # ボックスのURDFファイルの内容
    box_urdf = """
    <robot name="box">
        <link name="box">
            <visual>
                <geometry>
                    <box size="20 0.05 0.005"/>
                </geometry>
            </visual>
            <collision>
                <geometry>
                    <box size="20 0.05 0.005"/>
                </geometry>
            </collision>
            <inertial>
                <geometry>
                    <box size="20 0.05 0.005"/>
                </geometry>
            </inertial>
        </link>
    </robot>
    """

    # スポーンするボックスの名前と位置を指定
    box_name = "beam"
    box_pose = Pose()
    box_pose.position.x = 0.0
    box_pose.position.y = 0.0
    box_pose.position.z = 0.1  # Gazebo内の高さを設定

    # Gazeboにボックスをスポーンするサービスを呼び出す
    try:
        spawn_model = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp = spawn_model(box_name, box_urdf, "/", box_pose, "world")
        rospy.loginfo("Spawned box with result: %s", resp.status_message)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

if __name__ == '__main__':
    try:
        spawn_box()
    except rospy.ROSInterruptException:
        pass
