#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

def callback(data):
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

    if lines is not None:
        for rho, theta in lines[:, 0]:
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a * rho
            y0 = b * rho
            x1 = int(x0 + 1000 * (-b))
            y1 = int(y0 + 1000 * (a))
            x2 = int(x0 - 1000 * (-b))
            y2 = int(y0 - 1000 * (a))
            cv2.line(line_image, (x1, y1), (x2, y2), (255, 255, 255), 2)

    # Display the image with detected lines
    cv2.imshow("Line Detection", line_image)
    cv2.waitKey(1)
def listener():
    rospy.init_node('camera_subscriber', anonymous=True)
    rospy.Subscriber("/ab_camera/image_raw", Image, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()