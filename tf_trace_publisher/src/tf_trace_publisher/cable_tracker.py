#!/usr/bin/env python
import rospy
import tf

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_srvs.srv import *
import numpy as np

# Give ourselves the ability to run a dynamic reconfigure server.
# from dynamic_reconfigure.server import Server as DynamicReconfigureServer
# from tf_trace_publisher.cfg import tf_Trace_PublisherConfig as ConfigType

class CableTracker():
	def __init__(self):


		# self.server = DynamicReconfigureServer(ConfigType, self.reconfigure)
		self.srv = rospy.Service('~fix_pose', Empty, self.fix_pose)
		self.cable_length = 1.0
		self.fixed_poses = []

		self.base_frame = "world"
		self.fix_tcp = "fix_tcp"
		self.end_frame = "supposed_tcp"

		# initalize
		self.trace = Marker()
		self.trace.id = rospy.get_param('~id', 1)
		self.trace.type= Marker.LINE_STRIP
		self.trace.action = Marker.MODIFY
		self.trace.scale.x = 0.1
		self.trace.scale.y = 0.1
		self.trace.scale.z = 0.1

		self.trace.color.r = 1
		self.trace.color.g = 0
		self.trace.color.b = 0
		self.trace.color.a = 1

		self.trace.header.frame_id = self.base_frame
		# self.trace.ns = self.fix_tcp

		self.tf_listener = tf.TransformListener()
		self.pub_markerLine = rospy.Publisher('visualization_marker', Marker, queue_size=10)

		try:
			# self.tf_listener.waitForTransform(
			# 	self.base_frame,
			# 	self.fix_tcp,
			# 	rospy.Time(0),
			# 	rospy.Duration(4.0))
            #
			# (trans,rot) = self.tf_listener.lookupTransform(
			# 	self.base_frame,
			# 	self.fix_tcp,
			# 	rospy.Time(0))

			#self.fixed_poses.append([0.5487073659896851, -0.04490622505545616, 0.7260761260986328])
			#self.fixed_poses.append([0.6585336327552795, 0.06950411200523376, 0.7238875031471252])
			#self.fixed_poses.append([0.7527252435684204, 0.06950411200523376, 0.7238875031471252])
			pass
		except:
			return


	def run(self):
		while not rospy.is_shutdown():
			if self.fixed_poses:
				try:
					(trans,rot) = self.tf_listener.lookupTransform(
						self.base_frame,
						self.fix_tcp,
						rospy.Time(0))

					remaining_length = self.cable_length

					self.trace.points = []
					for i, fixture_point in enumerate(self.fixed_poses):
						# print fixture_point
						if i < len(self.fixed_poses)-1:
							remaining_length -= np.linalg.norm(np.array(self.fixed_poses[i])-np.array(self.fixed_poses[i+1]))

						p = Point()
					 	p.x = fixture_point[0]
					 	p.y = fixture_point[1]
					 	p.z = fixture_point[2]
						#print p
					 	self.trace.points.append(p)

					# print "trans",trans
					# print remaining_length

					# letzer punkt
					end_point = remaining_length * ((np.array(trans) - np.array(self.fixed_poses[-1])) / np.linalg.norm(np.array(trans) - np.array(self.fixed_poses[-1]))) + np.array(self.fixed_poses[-1])
					# print "end", end_point

					p = Point()
				 	p.x = end_point[0]
				 	p.y = end_point[1]
				 	p.z = end_point[2]
					self.trace.points.append(p)

					if len(self.trace.points) > 0:
						self.pub_markerLine.publish(self.trace)

				except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
					rospy.logwarn("Transformation Lookup failed")
					rospy.sleep(rospy.Duration(1))
					continue
			rospy.sleep(0.5)

	# # Create a callback function for the dynamic reconfigure server.
	# def reconfigure(self, config, level):
	# 	# Fill in local variables with values received from dynamic reconfigure clients (typically the GUI).
    #
	# 	self.trace.header.frame_id = config["tf_parent"]
	# 	self.trace.ns = config["tf_child"]
    #
	# 	self.trace.scale.x = config["scale"]
	# 	self.trace.scale.y = config["scale"]
	# 	self.trace.scale.z = config["scale"]
    #
	# 	self.trace.color.r = config["color_R"]
	# 	self.trace.color.g = config["color_G"]
	# 	self.trace.color.b = config["color_B"]
	# 	self.trace.color.a = config["color_A"]
    #
	# 	self.maxLen = config["length"]
	# 	self.rateTime = config["rate"]
	# 	self.rate = rospy.Rate(self.rateTime)
    #
	# 	config["tf_parent"] = self.trace.header.frame_id
	# 	config["tf_child"] = self.trace.ns
	# 	config["scale"] = self.trace.scale.x
	# 	config["color_R"] = self.trace.color.r
	# 	config["color_G"] = self.trace.color.g
	# 	config["color_B"] = self.trace.color.b
	# 	config["color_A"] = self.trace.color.a
    #
	# 	config["length"] = self.maxLen
	# 	config["rate"] = self.rateTime
    #
	# 	rospy.loginfo("Configuration changed!")
	# 	return config

	def fix_pose(self, req):
		try:
			(trans,rot) = self.tf_listener.lookupTransform(
				self.base_frame,
				self.fix_tcp,
				rospy.Time(0))
			self.fixed_poses.append(trans)

			return EmptyResponse()
		except:
			return

if __name__ == '__main__':

	rospy.init_node('CableTracker', anonymous=False)
	node = CableTracker()
	try:
		node.run()
	except rospy.ROSInterruptException: pass
