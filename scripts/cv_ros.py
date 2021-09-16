#!/usr/bin/env python2

import rospy
import cv2
import numpy as np
import sys

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def webcam_pub():
    pub = rospy.Publisher('/camera_rect/image_rect', Image, queue_size = 1)
    rospy.init_node('webcam_pub', anonymous = True)
    rate = rospy.Rate(60)

    cam = cv2.VideoCapture(0)

    if not cam.isOpened():
        sys.stdout.write("Webcam is not available")
        return -1

    while not rospy.is_shutdown():
        ret, frame = cam.read()
        bridge = CvBridge()
        msg = bridge.cv2_to_imgmsg(frame, encoding = "bgr8")

        if ret:
            rospy.loginfo("Capturing image")
        
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        webcam_pub()
    except rospy.ROSInterruptException:
        pass
        