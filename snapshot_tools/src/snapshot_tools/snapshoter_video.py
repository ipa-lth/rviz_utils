#! /usr/bin/python
# Copyright (c) 2017, Lorenz Halt, Fraunhofer IPA

# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html

import rospy
import re
import datetime as dt
import os

# ROS Image message
from sensor_msgs.msg import Image
from std_srvs.srv import Empty
from snapshot_tools.srv import String

from snapshot_tools.snapshoter import decode_filename

# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2

# Instantiate CvBridge
bridge = CvBridge()

take_image = False
filename = ""
cnt = 0
out = None
throttle_cnt = 0

def image_callback(msg):
    global take_image, filename, out, throttle_cnt
    if take_image:
        if throttle_cnt%(30*5) == 0:
            rospy.loginfo("Recording {} sec".format(throttle_cnt/30))
        throttle_cnt += 1
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError, e:
            print(e)
        else:
            # Save your OpenCV2 image as a avi file
            if out is None:
                filename = decode_filename(filename)
                fourcc = cv2.VideoWriter_fourcc(*'XVID')
                out = cv2.VideoWriter(filename, fourcc, 30.0, (msg.width, msg.height))
            out.write(cv2_img)
    else:
        if out is not None:
            out = None
            rospy.loginfo("Stop Recording {} after {} sec".format(filename, throttle_cnt/30.0))
            throttle_cnt = 0

def empty_srv_cb(req):
    global take_image
    take_image = False
    return []

def reset_cb(req):
    global cnt
    cnt = 0
    return []

def string_srv_cb(req):
    global take_image, filename
    if req.str.endswith(('.avi')):
        filename = req.str
    else:
        filename = "{}.avi".format(req.str)
    take_image = True
    return []

def main():
    global filename
    rospy.init_node('snapshoter_video')
    # Define your image topic
    image_topic = rospy.get_param("~image", "/image_raw")
    # Set up your subscriber and define its callback

    s = rospy.Service('~stop', Empty, empty_srv_cb)
    s = rospy.Service('~start', String, string_srv_cb)
    s = rospy.Service('~reset_cnt', Empty, reset_cb)

    rospy.Subscriber(image_topic, Image, image_callback)
    # Spin until ctrl + c
    rospy.loginfo("Ready to start recording")
    rospy.spin()

if __name__ == '__main__':
    main()
