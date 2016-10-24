#!/usr/bin/env python
import rospy
import cv2
    
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

class publish_image:

    def __init__(self):
        self.image_pub = rospy.Publisher("image",Image, queue_size=10)
        self.bridge = CvBridge()
        self.current_image_path  = rospy.get_param('~img_path')
        self.lookup_table  = rospy.get_param('~lookup', {})
        self.ignore_not_found  = rospy.get_param('~ignore_not_found', False)
        self.string_sub = rospy.Subscriber("image_trigger", String, self.cb)
        self.process_image()
        
    def cb(self, data):
        if data.data in self.lookup_table.keys():
            self.current_image_path = self.lookup_table[data.data] # This is not threadsafe, but works
        else:
            if not self.ignore_not_found:
                self.current_image_path = data.data # This is not threadsafe, but works
            else:
                rospy.logwarn("Ignored value '%s'. Not in lookup table", data.data)

    def process_image(self):
        r = rospy.Rate(24) #24hz
        latch_counter = 0

        while not rospy.is_shutdown():
            cv_image = cv2.imread(self.current_image_path, cv2.IMREAD_COLOR)
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image,"bgr8"))
                latch_counter = 0
            except:
                if latch_counter == 0:
                    rospy.logerr("no valid picture path: '%s'", self.current_image_path) # get message with the first appearance
                latch_counter+=1
                if (latch_counter > 3 * 24): # toggle message every three seconds [sec * 1/sec]
                    latch_counter = 0
            r.sleep()

if __name__ == '__main__':
    rospy.init_node('publish_image')
    pc = publish_image()
