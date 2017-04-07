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

# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2

# Instantiate CvBridge
bridge = CvBridge()

take_image = False
filename = ""
cnt = 0

def decode_filename(filename):
    global cnt
    # Replace number with cnt
    name = filename.replace("/{number/}", "{}".format(cnt))
    name = name.replace("{number}", "{}".format(cnt))
    cnt += 1

    # Find timestamp and replace according to formating
    for matchObj in re.finditer("{timestamp(.*?)}" ,name):
        #print "1:", matchObj.group()
        if matchObj.group(1) != "": # found custom formating
            #print "2:", matchObj.group(1)
            # Replace timestamp according to input parameters
            name = name.replace(matchObj.group(), "{}".format(dt.datetime.now().strftime(matchObj.group(1))))
        else:
            # Replace timestamp according to predefined (general) parameters
            name = name.replace(matchObj.group(),
                                "{}".format(
                                    dt.datetime.now().strftime(
                                        "%Y-%m-%d_%H:%M:%S")))

    # Change ~ to user
    name = os.path.expanduser(name)

    # Check if folder exists, otherwise create it
    path = os.path.dirname(name)
    if not os.path.isdir(path):
        rospy.loginfo("Creating new folder: {}".format(path))
        os.makedirs(path)

    rospy.logdebug("Decode Name: {} -> {}".format(filename, name))
    return name


def image_callback(msg):
    global take_image, filename
    if take_image:
        rospy.loginfo("Take snapshot")
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError, e:
            rospy.logerror("{}".format(e))
        else:
            # Save your OpenCV2 image as a jpeg
            name = decode_filename(filename)
            cv2.imwrite(name, cv2_img)
            rospy.loginfo("Saved image: {} ".format(name))
        finally:
            take_image = False

def empty_srv_cb(req):
    global take_image, filename
    filename = rospy.get_param("~filename", "~/Pictures/{timestamp/}-image_/{number/}.jpg")
    take_image = True
    return []

def reset_cb(req):
    global cnt
    cnt = 0
    return []

def string_srv_cb(req):
    global take_image, filename
    if req.str.endswith(('.jpg', '.jpeg')):
        filename = req.str
    else:
        filename = "{}.jpg".format(req.str)
    take_image = True
    return []

def main():
    global filename
    rospy.init_node('snapshoter')
    # Define your image topic
    image_topic = rospy.get_param("~image", "/usb_cam/image_raw")
    filename = rospy.get_param("~filename", "~/Pictures/{timestamp/}-image_/{number/}.jpg")
    # Set up your subscriber and define its callback

    s = rospy.Service('~save_default', Empty, empty_srv_cb)
    s = rospy.Service('~save', String, string_srv_cb)
    s = rospy.Service('~reset_cnt', Empty, reset_cb)

    rospy.Subscriber(image_topic, Image, image_callback)
    # Spin until ctrl + c
    rospy.loginfo("Ready to take images")
    rospy.spin()

if __name__ == '__main__':
    main()
