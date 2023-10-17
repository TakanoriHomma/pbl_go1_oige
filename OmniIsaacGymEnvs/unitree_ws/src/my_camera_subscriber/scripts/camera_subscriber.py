#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge
import cv2
import numpy as np

def callback(data):
    global angle_pub

    bridge = CvBridge()
    try:
        # Convert ROS Image to OpenCV image
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except Exception as e:
        rospy.logerr("Could not convert image: %s", e)
        return

    # Convert the image to grayscale
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    # Use Canny edge detection
    edges = cv2.Canny(gray, 50, 150, apertureSize=3)

    # Detect lines using Hough Line Transform
    lines = cv2.HoughLines(edges, 1, np.pi/180, 200)

    # Create a blank black image
    line_image = np.zeros_like(cv_image)

    angle_rad = 0.0  # Default angle

    if lines is not None:
        # Find the longest line
        longest_line = None
        max_length = 0.0
        for rho, theta in lines[:, 0]:
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a * rho
            y0 = b * rho
            x1 = int(x0 + 1000 * (-b))
            y1 = int(y0 + 1000 * (a))
            x2 = int(x0 - 1000 * (-b))
            y2 = int(y0 - 1000 * (a))
            length = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
            if length > max_length:
                longest_line = (x1, y1, x2, y2, theta)
                max_length = length

        # Draw the longest line
        if longest_line is not None:
            cv2.line(line_image, (longest_line[0], longest_line[1]), (longest_line[2], longest_line[3]), (255, 255, 255), 2)
            
            # Compute the angle with respect to the vertical centerline
            if 0 < longest_line[4] < np.pi/2:
                angle_rad = -1.0*(longest_line[4])
            elif np.pi/2 < longest_line[4] < np.pi:
                angle_rad = -1.0*(longest_line[4] - np.pi)
            else:
                angle_rad = 0.0  # The line is parallel to the vertical centerline

            angle_deg = np.degrees(angle_rad)
            
            # Display the angle on the image
            text = "{:.1f} degrees".format(angle_deg)
            cv2.putText(line_image, text, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)

    # Draw the vertical centerline
    height, width, channels = line_image.shape
    center_x = width // 2
    cv2.line(line_image, (center_x, 0), (center_x, height), (0, 255, 0), 2)  # White line, 2 pixels wide

    # Display the image with detected lines, centerline, and angle
    cv2.imshow("Line Detection", line_image)
    cv2.waitKey(1)

    # Publish the angle in radians
    angle_pub.publish(angle_rad)

def listener():
    global angle_pub

    rospy.init_node('camera_subscriber', anonymous=True)
    
    # Create a publisher for the angle
    angle_pub = rospy.Publisher("/detected_angle", Float64, queue_size=10)

    rospy.Subscriber("/ab_camera/image_raw", Image, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()

