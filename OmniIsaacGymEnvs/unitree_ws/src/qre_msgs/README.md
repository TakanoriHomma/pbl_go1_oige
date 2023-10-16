# qre_msgs

This package has messages and services for _qre_ros_ package. 

## Messages

__BodyPose__

This message is used to body pose of the robot. Content of the message is:

```
float32 roll
float32 pitch
float32 yaw
float32 body_height
```

__JointCMD__

This message if used in low level control and contains different values for each joint. Message content is:

```
float32[] q   # Position
float32[] dq  # Velocity
float32[] tau # Torque
float32[] Kp
float32[] Kd
```

Pay attention to the sequence of the joints. Legs are followed from _Front Rght, Front Left, Rare Right, Right Left_ and the joints _Hip, Thight, Calf_.

["FR_hip_joint", "FR_thigh_joint", "FR_calf_joint", "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint", "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint", "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"]

## Services

__SetBodyPose__

This service is intended to set robot pose. It takes input _BodyPose_ messageand return nothing.

__SetControl__

Set control service to be used in low level mode. Service content is:

```
string control # Position, Velocity, Torque
---
bool success
string message
```