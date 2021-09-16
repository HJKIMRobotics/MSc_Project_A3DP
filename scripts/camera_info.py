#!/usr/bin/env python2

import rospy
import cv2

from sensor_msgs.msg import CameraInfo

rospy.init_node('camera_info', anonymous = True)

# def callback(data):
#     q = CameraInfo()
#     print(q)

pub = rospy.Publisher('/camera_rect/camera_info', CameraInfo, queue_size = 10)
rate = rospy.Rate(60)

while not rospy.is_shutdown():
    q = CameraInfo()
    # q.header.seq = 30
    # q.header.stamp.secs = 0
    # q.header.stamp.nsecs = 0
    q.header.frame_id = 'usb_cam'
    q.width = 640
    q.height = 480
    # q.distortion_model = 'plumb_bob'
    q.D = [0.23456741110616952, -0.3256843736125957, -0.044188638610536284, -0.002496131736214364, 0.0]
    q.K = [660.0951202969148, 0.0, 333.97684937287676, 0.0, 669.8032433052235, 186.69788457886807, 0.0, 0.0, 1.0]
    q.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    q.P = [701.0924072265625, 0.0, 332.6089871924087, 0.0, 0.0, 686.8612670898438, 171.58060330156331, 0.0, 0.0, 0.0, 1.0, 0.0]
    q.binning_x = 0
    q.binning_y = 0
    q.roi.x_offset = 0
    q.roi.y_offset = 0
    q.roi.width = 0
    q.roi.height = 0
    q.roi.do_rectify = False
    pub.publish(q)
    # rate.sleep()

# def listener():
#     rospy.Subscriber('/camera_rect/camera_info', CameraInfo, callback)

# if __name__ = '__main__':
#     listener()
#     rospy.spin()


