#!/usr/bin/env python

import rospy
import random

from std_msgs.msg import String

#comment
def text_publisher():
	rospy.init_node('string_node',anonymous=True)
	pub=rospy.Publisher('/info',String, queue_size=10)
	r = rospy.Rate(0.5)
	while not rospy.is_shutdown():
		text=['Run','Faster','Even Faster','Stop']
		i = random.randint(0,3)
		rospy.loginfo(text[i])
		pub.publish(text[i])
		r.sleep()


if __name__ == '__main__':
	try:
		text_publisher()
	except rospy.ROSInterruptException: pass

