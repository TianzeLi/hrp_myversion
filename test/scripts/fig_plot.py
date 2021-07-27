#! /usr/bin/python3

'''
 Plot robot position stored in ROS bag receive in real-time 
 Enabled by matplotlib for generating figures in the report.
 
 Procedure:
   1. If from recorded bag, call rosbag API to extract the data. 
   2. Plot with the matplotlib.

 Elements in source: topic name, color, angle to rotate to align with
                     ENU, source name for ploting.
 Elements in the figure for publish: axis label, legend, title, grid.

 Note: 
   1. Python3 does not support LZ4, then we get the following warning: 
      Failed to load Python extension for LZ4 support.
   
'''

import rosbag
import rospy
import matplotlib.pyplot as plt
import numpy as np
from scipy.ndimage import rotate
from os.path import expanduser
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import NavSatFix


class BagPlot():

    def __init__(self):

        # Configuration parameters.
        self.plot_from_bag = False
        self.plot_3D = False

        # topic_list = ["/odom", "/odometry/filtered"]
        # topic_list = ["/odometry_filtered", "/odom"]
        # type_list = ["Odometry", "Odometry"]
        # color_list = ['orange', 'chocolate']
        # angle_list = [1.57-0.3, 0.0]
        # source_list = ["Encoder", "Encoder not rotated"]

        # topic_list = ["/odometry/filtered"]
        topic_list = ["/odometry/filtered"]
        type_list = ["Odometry"]
        color_list = ["green"]
        angle_list = [0.0]
        source_list = ["Fused encoders and IMU"]

        topic_list = ["/rtabmap/odom"]
        type_list = ["Odometry"]
        color_list = ["green"]
        angle_list = [1.57-0.3]
        source_list = ["Visual odometry"]

        self.xlim = [-40, 60]
        self.ylim = [-20, 80] 

        # # GNSS raw
        # topic_list = ["/gnss_left/fix", "/gnss_right/fix", "/GPSfix"]
        # type_list = ["NavSatFix", "NavSatFix", "NavSatFix"]
        # color_list = ["orange", "orangered", "orchid"]
        # angle_list = [0.0, 0.0, 0.0]
        # source_list = ["Left GNSS", "Right GNSS", "Original GNSS"]
        
        # # GNSS ENU xy 
        # topic_list = ["/gnss_left/pose", "/gnss_right/pose", 
        #             "/GPSfix_processed"]
        # type_list = ["PoseWithCovarianceStamped", "PoseWithCovarianceStamped", 
        #             "PoseWithCovarianceStamped"]
        # color_list = ["orange", "orangered", "orchid"]
        # angle_list = [0.0, 0.0, 0.0]
        # source_list = ["Left GNSS", "Right GNSS", "Original GNSS"]

        if (self.plot_from_bag): 
            #  Bag location and read.
            home = expanduser("~")
            dir_name = home + "/bags/lawn/"
            filename = "lawn_one_lap_20201122.bag"
            self.bag = rosbag.Bag(dir_name + filename)

            for topic, color, angle, source in zip(topic_list, color_list, 
                                           angle_list, source_list): 
                self.topic_plot(topic, color, angle, source)
            plt.show()
            self.bag.close()

        else:
            # Real time plot configurations.
            self.plot_per_points = 3
            self.point_size = 3
            rospy.init_node('bag_plot', log_level=rospy.DEBUG)

            serial = 0
            self.counter = []
            self.legend_added = []
            for topic, topic_type, color, angle, source in zip(
                                        topic_list, type_list, color_list, 
                                        angle_list, source_list):
                rospy.logdebug("We have topic {} of type {} in color {}."
                        .format(topic, topic_type, color)) 
                if (topic_type == "Odometry"): 
                    rospy.Subscriber(topic, Odometry, self.odom_callback,
                                    (serial, color, source, angle))
                if (topic_type == "PoseWithCovarianceStamped"): 
                    rospy.Subscriber(topic, PoseWithCovarianceStamped, 
                                    self.pose_callback, 
                                    (serial, color, source, angle))
                if (topic_type == "NavSatFix"): 
                    rospy.Subscriber(topic, NavSatFix, 
                                    self.gnssfix_callback, 
                                    (serial, color, source, angle))
                serial += 1
                self.counter.append(0)
                self.legend_added.append(False)

            plt.ion()
            plt.axis('scaled')
            plt.xlabel('X coordinate')
            plt.ylabel('Y coordinate')
            plt.title('Estimation in ENU frame')
            # plt.legend(topic_list, source_list)
            plt.legend()
            plt.grid(True)
            plt.xlim(self.xlim)
            plt.ylim(self.ylim)
            rospy.logdebug("In total {} topics.".format(serial))
            # rospy.spin()
            plt.show(block=True) 


    def rotate_xypair(self, x, y, theta):
        x_rotated = x*np.cos(theta) - y*np.sin(theta)
        y_rotated = x*np.sin(theta) + y*np.cos(theta)
        return x_rotated, y_rotated

    def topic_plot(self, topic_name, color, angle, source): 
        """Plot from data stored in rosbag."""

        odom_x = []
        odom_y = []
        for topic, msg, t in self.bag.read_messages(topics=topic_name):
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            if(angle != 0.0):
                x, y = self.rotate_xypair(x, y, angle)
            odom_x.append(x)
            odom_y.append(y)

        plt.plot(odom_x, odom_y, color=color, label=source)
        plt.axis('scaled')
        plt.legend()
        plt.xlabel('X coordinate')
        plt.ylabel('Y coordinate')
        plt.title('Estimation in ENU frame')
        plt.grid(True)

    def plot_2D(self, x, y, color, source, serial):
        """Plot x and y coordinates."""
        plt.plot(x, y, color=color, label=source, 
                marker=".", markersize=self.point_size)
        plt.axis('square')
        plt.xlim(self.xlim)
        plt.ylim(self.ylim)
        plt.draw()
        if not self.legend_added[serial]:
            plt.legend()
            self.legend_added[serial] = True
        rospy.logdebug("For {} to plot {}, {}."
                        .format(source, x, y))

    def plot_3D(self, x, y, z, color, source):
        """Plot xyz coordinates."""
        pass

    def odom_callback(self, msg, args):
        serial = args[0]
        color = args[1]
        source = args[2]
        angle = args[3]

        self.counter[serial] += 1
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        if(angle != 0.0):
            x, y = self.rotate_xypair(x, y, angle)
        if (self.plot_3D):
            self.plot_3D(x, y, z, color, source)
        elif (self.counter[serial] % self.plot_per_points == 0):
            self.plot_2D(x, y, color, source, serial)

    def pose_callback(self, msg, args):

        serial = args[0]
        color = args[1]
        source = args[2]
        angle = args[3]

        self.counter[serial] += 1
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        if(angle != 0.0):
            x, y = self.rotate_xypair(x, y, angle)
        if (self.plot_3D):
            self.plot_3D(x, y, z, color, source)
        elif (self.counter[serial] % self.plot_per_points == 0):
            self.plot_2D(x, y, color, source, serial)

    def gnssfix_callback(self, msg, args):

        serial = args[0]
        color = args[1]
        source = args[2]
        angle = args[3]

        self.counter[serial] += 1
        x = msg.longitude
        y = msg.latitude
        z = msg.altitude

        if(angle != 0.0):
            x, y = self.rotate_xypair(x, y, angle)
        if (self.plot_3D):
            self.plot_3D(x, y, z, color, source)
        elif (self.counter[serial] % self.plot_per_points == 0):
            self.plot_2D(x, y, color, source, serial)


if __name__ == '__main__':
    BagPlot()