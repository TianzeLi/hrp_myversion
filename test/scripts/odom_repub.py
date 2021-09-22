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
        to_pub.header = msg.header
        to_pub.child_frame_id = "base_link"
        to_pub.pose.pose = msg.pose.pose
        to_pub.twist.twist = msg.twist.twist

        # to_pub.header.stamp = rospy.Time.now()

        to_pub.twist.covariance[0] = 0.08*0.08
        to_pub.twist.covariance[7] = 1e-8
        to_pub.twist.covariance[14] = 1e-4
        to_pub.twist.covariance[21] = 1e-4
        to_pub.twist.covariance[28] = 1e-4
        to_pub.twist.covariance[35] = 0.07*0.07

        self.sim_pose_pub.publish(to_pub)


if __name__ == '__main__':
    rospy.init_node('gnss_simu')
    OdomRepub()
    rospy.spin()