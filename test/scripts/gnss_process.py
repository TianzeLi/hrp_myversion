#! /usr/bin/python3

"""
 Refine embedded origin GPS measurement for fusion.

"""

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import NavSatFix


class GNSSProcess():

	def __init__(self):

		topic_name = "/GPSfix"
		self.x_scale_factor = 0.5
		self.y_scale_factor = 1.0
		self.do_estimate_initial = True
		
		self.degree2meter = 111320
		self.initial_latitude = 59.40455 
		self.initial_longitude = 17.94965
		self.latitude_stdev = 5.0
		self.longitude_stdev = 5.0

		if (self.do_estimate_initial):
			self.initial_latitude = 0.0 
			self.initial_longitude = 0.0
			self.initial_stack_number = 3
			self.number_count = 0

		self.pose_pub = rospy.Publisher((topic_name + "_processed"), PoseWithCovarianceStamped, queue_size=10)
		rospy.Subscriber(topic_name, NavSatFix, self.gpsfix_callback)

	def gpsfix_callback(self, msg):
		to_pub = PoseWithCovarianceStamped()
		to_pub.header = msg.header
		# to_pub.header.stamp = rospy.Time.now()

		if (self.do_estimate_initial):
			if (self.initial_stack_number > self.number_count):
				self.initial_latitude += msg.latitude
				self.initial_longitude += msg.longitude
				self.number_count += 1
			else:
				self.initial_latitude = self.initial_latitude / self.initial_stack_number
				self.initial_longitude = self.initial_longitude / self.initial_stack_number
				self.do_estimate_initial = False

		if not self.do_estimate_initial:
			x,y = self.compute_local_corrdinate(msg.latitude, msg.longitude)
			to_pub.pose.pose.position.x = x * self.x_scale_factor
			to_pub.pose.pose.position.y = y * self.y_scale_factor
			# to_pub.pose.pose.covariance[0] = msg.position_covariance[0]
			# to_pub.pose.pose.covariance[9] = msg.position_covariance[4]
			to_pub.pose.covariance[0] = self.longitude_stdev
			to_pub.pose.covariance[9] = self.latitude_stdev
			self.pose_pub.publish(to_pub)

	def compute_local_corrdinate(self, lati, longi):
		x_local = self.degree2meter*(longi - self.initial_longitude)
		y_local = self.degree2meter*(lati - self.initial_latitude)
		return x_local, y_local


if __name__ == '__main__':
	rospy.init_node('gnss_simu')
	GNSSProcess()
	rospy.spin()