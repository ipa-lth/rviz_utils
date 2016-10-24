#!/usr/bin/env python
import rospy
import cv2
    
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class publish_image:

    def __init__(self):
        self.image_pub = rospy.Publisher("image", Image, queue_size=10)
        self.bridge = CvBridge()
        self.process_image()
        
    def process_image(self):
        r = rospy.Rate(24) #24hz

        while not rospy.is_shutdown():
            path = rospy.get_param('~img_path')
            cv_image = cv2.imread(path, cv2.IMREAD_COLOR)
            print cv_image
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image,"bgr8"))
            except:
                rospy.logerr("no valid picture path")
            r.sleep()

if __name__ == '__main__':
    rospy.init_node('publish_image')
    pc = publish_image()
