#!/usr/bin/env python
import rospy
import tf

from std_msgs.msg import String
from visualization_msgs.msg import Marker

# Class represents a Node, that subscribes a String message over work_text topic
# and publishes a Marker (type TEXT_VIEW_FACING) to the visualization_marker topic
# of rviz.

class TextPublisher():
	def __init__(self):
		# initalize

		self.__textMarker = self.create_textMarker()
		self.__sub_string=rospy.Subscriber('/info',String,self.string_callback)
		self.__pub_marker=rospy.Publisher('visualization_marker',Marker, queue_size=10)
		self.update_params()
		self.__text='Initialization'
		self.build_marker_up(self.__text,self.__textMarker)

	def create_textMarker(self):
		marker = Marker()
		marker.header.frame_id = rospy.get_param('~frame',"/text_frame")
		marker.id = rospy.get_param('~id', 1)
		marker.type= Marker.TEXT_VIEW_FACING
		marker.action = Marker.MODIFY	
		return marker

	def set_Text(self,data):
		self.__text=data

	def get_Text(self):
		return self.__text

	def update_params(self):
		self.__pos_x=rospy.get_param('~frame_Position_X',1)
		self.__pos_y=rospy.get_param('~frame_Position_Y',1)
		self.__pos_z=rospy.get_param('~frame_Position_Z',1)
		self.__fade_rate = rospy.get_param('~fade_Rate',0.1)
		self.__textMarker.scale.z = rospy.get_param('~scale_Z', 0.5)
		self.__textMarker.color.r = rospy.get_param('~color_Red',0.0)
		self.__textMarker.color.g = rospy.get_param('~color_Green',1.0)
		self.__textMarker.color.b = rospy.get_param('~color_Blue',0.0)


	def string_callback(self,data):
		# rospy.loginfo("I heard %s",data)
		if data.data =='':
			data.data='Busy!'
		self.build_marker_down(self.__text,self.__textMarker)
		self.__text = data.data
		self.build_marker_up(self.__text,self.__textMarker)


	def build_marker_up(self,data,marker):
		marker.text = data
		for a in range(0,11,1):
			marker.color.a = float(a)/10
			self.pub_text()
			rospy.sleep(self.__fade_rate)

	def build_marker_down(self,data,marker):
		marker.text = data
		for a in range(10,-1,-1):
			marker.color.a = float(a)/10
			self.pub_text()
			rospy.sleep(self.__fade_rate)

	def pub_text(self):
		self.__pub_marker.publish(self.__textMarker)


if __name__ == '__main__':

	rospy.init_node('text_Publisher', anonymous=True)
	try:
		node = TextPublisher()
		while not rospy.is_shutdown():
			node.update_params()
			rospy.sleep(1)
	except rospy.ROSInterruptException:
		pass