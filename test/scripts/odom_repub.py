#! /usr/bin/python3

"""
 Refine or manipulate encoder measurements for fusion. 
 Initially to align the timestamp.
 
"""

import rospy
from nav_msgs.msg import Odometry


class OdomRepub():

	def __init__(self):

		topic_name = "/odom"

		self.sim_pose_pub = rospy.Publisher((topic_name + "_repub"), Odometry, queue_size=10)
		rospy.Subscriber(topic_name, Odometry, self.pose_callback)

	def pose_callback(self, msg):
		to_pub = Odometry()
		to_pub = msg
		to_pub.header.stamp = rospy.Time.now()
		self.sim_pose_pub.publish(to_pub)


if __name__ == '__main__':
	rospy.init_node('gnss_simu')
	OdomRepub()
	rospy.spin()