#!/usr/bin/env python
import rospy

from visualization_msgs.msg import Marker

# Give ourselves the ability to run a dynamic reconfigure server.
# from dynamic_reconfigure.server import Server as DynamicReconfigureServer
# from ipa325_itasc_commons.cfg import textMarkerPublisher_paramsConfig as ConfigType

class TextPublisher():
	def __init__(self):

		# initalize
		textMarker = Marker()
		textMarker.header.frame_id = "/textFrame"
		textMarker.id = rospy.get_param('~id', 1)
		textMarker.type= Marker.TEXT_VIEW_FACING
		textMarker.action = Marker.MODIFY
		# textMarker.ns = tf_child

		# textMarker.scale.x = rospy.get_param('~scale_X', 0.01)
		# textMarker.scale.y = rospy.get_param('~scale_Y', 0.01)
		textMarker.scale.z = rospy.get_param('~scale_Z', 0.1)

		textMarker.text = 'x_'

		textMarker.color.r = rospy.get_param('~color_Red', 0.0)
		textMarker.color.g = rospy.get_param('~color_Green', 1.0)
		textMarker.color.b = rospy.get_param('~color_Blue', 0.0)
		textMarker.color.a = rospy.get_param('~color_Alpha', 1.0)

		rate = rospy.Rate(rospy.get_param('~rate', 5))

		pub_markerLine = rospy.Publisher('visualization_marker', Marker)

		cnt = 0
		maxCnt = 10
		self.text = 'x_'
		while not rospy.is_shutdown():
			if cnt < maxCnt:
				textMarker.action = Marker.MODIFY
				self.text += str(cnt)
				cnt += 1
			else:
				self.text = 'x_'
				textMarker.action = Marker.DELETE
				cnt = 0
				

			textMarker.text = self.text

			pub_markerLine.publish(textMarker)

			rate.sleep()

if __name__ == '__main__':

	rospy.init_node('tf_textMarker_Publisher', anonymous=True)
	try:
		node = TextPublisher()
	except rospy.ROSInterruptException: pass
