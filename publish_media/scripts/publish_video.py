#!/usr/bin/env python
import rospy
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class publish_video():

    def __init__(self):
        self.image_pub = rospy.Publisher("video",Image, queue_size=100)
        self.bridge = CvBridge()
        self.process_video()
        
    def process_video(self):
        while not rospy.is_shutdown():
            b_loop = True
            r = rospy.Rate(24)

            while b_loop and not rospy.is_shutdown():
                path = rospy.get_param("~vid_path")
                b_loop = rospy.get_param("~bool_loop")
                cap = cv2.VideoCapture(path)

                while(cap.isOpened()) and not rospy.is_shutdown():
                    ret, frame = cap.read()

                    try:
                        self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
                    except:
                        if not b_loop:
                            rospy.signal_shutdown('All done.')
                        break

                    r.sleep()
        cap.release()

if __name__ == '__main__':
    rospy.init_node('publish_video')
    pc = publish_video()
