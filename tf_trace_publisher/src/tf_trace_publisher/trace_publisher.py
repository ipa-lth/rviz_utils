#!/usr/bin/env python
import rospy
import tf

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_srvs.srv import *


# Give ourselves the ability to run a dynamic reconfigure server.
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from tf_trace_publisher.cfg import tf_Trace_PublisherConfig as ConfigType

class TfTracePublisher():
	def __init__(self):


		# initalize
		self.trace = Marker()
		self.trace.id = rospy.get_param('~id', 1)
		self.trace.type= Marker.LINE_STRIP
		self.trace.action = Marker.MODIFY

		self.server = DynamicReconfigureServer(ConfigType, self.reconfigure)
		self.srv = rospy.Service('~reset', Empty, self.reset_trace)

		tf_listener = tf.TransformListener()
		pub_markerLine = rospy.Publisher('visualization_marker', Marker, queue_size=10)

		while not rospy.is_shutdown():
			try:
				(trans,rot) = tf_listener.lookupTransform(self.trace.header.frame_id, self.trace.ns, rospy.Time(0))

				if True:# len(trace.points) == 0 or not (trace.points[len(trace.points)-1].x != trans[0] and trace.points[len(trace.points)-1].y != trans[1] and trace.points[len(trace.points)-1].z != trans[2]):
					p1 = Point()
					p1.x = trans[0]
					p1.y = trans[1]
					p1.z = trans[2]
					while len(self.trace.points) > self.maxLen:
						self.trace.points.pop(0)
					self.trace.points.append(p1)

				if len(self.trace.points) > 0:
					pub_markerLine.publish(self.trace)

			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				rospy.logwarn("Transformation Lookup failed")
				rospy.sleep(rospy.Duration(1))
				continue
			self.rate.sleep()

	# Create a callback function for the dynamic reconfigure server.
	def reconfigure(self, config, level):
		# Fill in local variables with values received from dynamic reconfigure clients (typically the GUI).

		self.trace.header.frame_id = config["tf_parent"]
		self.trace.ns = config["tf_child"]

		self.trace.scale.x = config["scale"]
		self.trace.scale.y = config["scale"]
		self.trace.scale.z = config["scale"]
		
		self.trace.color.r = config["color_R"]
		self.trace.color.g = config["color_G"]
		self.trace.color.b = config["color_B"]
		self.trace.color.a = config["color_A"]

		self.maxLen = config["length"]
		self.rateTime = config["rate"]
		self.rate = rospy.Rate(self.rateTime)

		config["tf_parent"] = self.trace.header.frame_id
		config["tf_child"] = self.trace.ns
		config["scale"] = self.trace.scale.x
		config["color_R"] = self.trace.color.r
		config["color_G"] = self.trace.color.g
		config["color_B"] = self.trace.color.b
		config["color_A"] = self.trace.color.a

		config["length"] = self.maxLen
		config["rate"] = self.rateTime

		rospy.loginfo("Configuration changed!")
		return config

	def reset_trace(self, req):
		del self.trace.points[:]
		return EmptyResponse()

if __name__ == '__main__':

	rospy.init_node('tf_Trace_Publisher', anonymous=True)
	try:
		node = TfTracePublisher()
	except rospy.ROSInterruptException: pass
