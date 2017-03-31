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
out = None
throttle_cnt = 0

def decode_filename(filename):
    global cnt
    # Replace number with cnt
    name = filename.replace("/{number/}", "{}".format(cnt))
    name = name.replace("{number}", "{}".format(cnt))
    cnt += 1

    # TODO: Make this as one regex

    # Find timestamp and replace according to formating
    matchObj = re.search("/{timestamp(.*?)/}" ,name)
    if matchObj: # found timestamp tag
        print "1:", matchObj.group()
        if matchObj.group(1) != "": # found custom formating 
            print "2:", matchObj.group(1)
            # Replace timestamp according to input parameters
            name = name.replace(matchObj.group(), "{}".format(dt.datetime.now().strftime(matchObj.group(1))))
        else:
            # Replace timestamp according to predefined (general) parameters
            name = name.replace(matchObj.group(),
                                "{}".format(
                                    dt.datetime.now().strftime(
                                        "%Y-%m-%d_%H:%M:%S")))

    matchObj = re.search("{timestamp(.*?)}" ,name)
    if matchObj: # found timestamp tag
        print "1:", matchObj.group()
        if matchObj.group(1) != "": # found custom formating 
            print "2:", matchObj.group(1)
            # Replace timestamp according to input parameters
            name = name.replace(matchObj.group(), "{}".format(dt.datetime.now().strftime(matchObj.group(1))))
        else:
            # Replace timestamp according to predefined (general) parameters
            name = name.replace(matchObj.group(),
                                "{}".format(
                                    dt.datetime.now().strftime(
                                        "%Y-%m-%d_%H:%M:%S")))

    rospy.logdebug("Decode Name: {} -> {}".format(filename, name))
    return name

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
