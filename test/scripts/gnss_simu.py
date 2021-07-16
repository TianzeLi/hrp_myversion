#! /usr/bin/python3

"""
 Refine or sent simultated GNSS measurements for fusion.
 
 One way is to subcscribe to an existing gnss topic and 
 then publish the processed measurement at re-specified
 frequency.

"""

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped


class GNSSSimu():

	def __init__(self):

		topic_name = "/GPSfix"
		pub_rate = 0.1 # Unit in Hz
		r = rospy.Rate(pub_rate)
		self.x_scale_factor = 1.0
		self.y_scale_factor = 1.0

		self.to_pub = PoseWithCovarianceStamped()

		sim_pose_pub = rospy.Publisher((topic_name + "_simu"), PoseWithCovarianceStamped, queue_size=10)
		rospy.Subscriber(topic_name, PoseWithCovarianceStamped, self.pose_callback)
		while not rospy.is_shutdown():
			sim_pose_pub.publish(self.to_pub)
			r.sleep()


	def pose_callback(self, msg):
		# rospy.loginfo("I heard %s",data.data)
		self.to_pub = msg
		# self.to_pub.header.stamp = rospy.Time.now()
		self.to_pub.pose.pose.position.x *= self.x_scale_factor
		self.to_pub.pose.pose.position.y *= self.y_scale_factor

		# self.to_pub.pose.covariance[0] = 30.0
		# self.to_pub.pose.covariance[7] = 30.0

		# x = -self.to_pub.pose.pose.position.y
		# y = self.to_pub.pose.pose.position.x

		# self.to_pub.pose.pose.position.x = x
		# self.to_pub.pose.pose.position.y = y



if __name__ == '__main__':
	rospy.init_node('gnss_simu')
	GNSSSimu()
	rospy.spin()