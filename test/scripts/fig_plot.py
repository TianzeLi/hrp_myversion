#! /usr/bin/python3

'''
 Plot robot position from data stored in ROS bag or received in real-time. 
 Using matplotlib for generating figures in the report.
 
 Note: 
    0. Elements in the figure for publish: axis label, legend, title, grid.
    1. Python3 does not support LZ4 (supported by rosbag), then we get the
       following warning: Failed to load Python extension for LZ4 support.
   
'''

import rosbag
import rospy
import matplotlib.pyplot as plt
import numpy as np
from math import sqrt
from os.path import expanduser
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import NavSatFix


class BagPlot():

    def __init__(self):

        # Overall configuration parameters.
        self.plot_from_bag = False
        self.plot_3D_trace = False
        self.plot_defined_trace = True
        self.plot_xy_covariance = True
        self.plot_yaw_covariance = True
        self.plot_covariance_in_stdev = True
        # Real time plot configurations.
        self.plot_per_points = 20
        self.point_size = 3
        
        # Figure scale.
        self.xlim = [-60, 60]
        self.ylim = [-20, 100] 
        self.zlim = [-12, 12] 

        # Configure the data to plot.
        topic_list = ["/odometry/filtered"]
        type_list = ["Odometry"]
        color_list = ['green']
        angle_list = [0.0]
        source_list = ["E.+IMU+GNSS"]

        # To plot the adaptive R matrix.
        # topic_list = ["/GPSfix_processed"]
        # type_list = ["PoseWithCovarianceStamped"]
        # color_list = ['green']
        # angle_list = [0.0]
        # source_list = ["Adaptive GNSS"]

        # # Visual-SLAM.
        # topic_list = ["/rtabmap/odom"]
        # type_list = ["Odometry"]
        # color_list = ["green"]
        # angle_list = [1.57-0.3]
        # source_list = ["Visual odometry"]

        # # GNSS raw fix data.
        # topic_list = ["/gnss_left/fix", "/gnss_right/fix", "/GPSfix"]
        # type_list = ["NavSatFix", "NavSatFix", "NavSatFix"]
        # color_list = ["orange", "orangered", "orchid"]
        # angle_list = [0.0, 0.0, 0.0]
        # source_list = ["Left GNSS", "Right GNSS", "Original GNSS"]
        
        # # GNSS ENU xy. 
        # topic_list = ["/gnss_left/pose", "/gnss_right/pose", 
        #             "/GPSfix_processed"]
        # type_list = ["PoseWithCovarianceStamped", "PoseWithCovarianceStamped", 
        #             "PoseWithCovarianceStamped"]
        # color_list = ["orange", "orangered", "orchid"]
        # angle_list = [0.0, 0.0, 0.0]
        # source_list = ["Left GNSS", "Right GNSS", "Original GNSS"]

        # Aimed path for reference.
        self.trace = [[0.00, 0.00],  
                      [3.80, 25.3],
                      [19.6, 22.6],
                      [27.0, 67.9],
                      [12.5, 73.4],
                      [-12.7, 60.5],
                      [-23.1, -0.8],
                      [-0.50, -4.2],
                      [-0.30, -1.6]]

        self.plot_total_num = 1
        if self.plot_xy_covariance:
            self.plot_total_num += 1
        if self.plot_yaw_covariance:
            self.plot_total_num += 1

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
        
            self.legend_added_xy_var = False
            self.legend_added_yaw_var = False

            self.last_x = 0.0
            self.last_y = 0.0
            self.length_travelled = 0.0

            fig = plt.figure()
            if self.plot_total_num ==2:
                fig_covar = plt.figure()
                self.ax_covar1 = fig_covar.add_subplot()
            if self.plot_total_num == 3:
                fig_covar = plt.figure()
                self.ax_covar1 = fig_covar.add_subplot(211)
                self.ax_covar2 = fig_covar.add_subplot(212)
            if self.plot_xy_covariance:
                # self.ax_covar1.set_xlabel('Distance travelled (m)')
                if self.plot_covariance_in_stdev:
                    self.ax_covar1.set_ylabel('Standard deviation')
                    self.ax_covar1.set_title('Standard deviation in x, y and yaw')
                else:
                    self.ax_covar1.set_ylabel('Variance')
                    self.ax_covar1.set_title('Variance in estimated x, y and yaw')
                self.ax_covar1.grid(True)
            if self.plot_yaw_covariance:
                self.ax_covar2.set_xlabel('Distance travelled (m)')
                if self.plot_covariance_in_stdev:
                    self.ax_covar2.set_ylabel('Standard deviation')
                #     self.ax_covar2.set_title('Standard deviation in estimated yaw')
                else:
                    self.ax_covar2.set_ylabel('Variance')
                #     self.ax_covar2.set_title('Variance in estimated yaw')
                self.ax_covar2.grid(True)

            if self.plot_3D_trace:
                self.ax = fig.add_subplot(projection='3d')
                self.ax.set_zlim(self.zlim)
                self.ax.set_box_aspect((1, 1, 0.2))
            else:
                self.ax = fig.add_subplot()
                self.ax.axis('scaled')

            plt.ion()
            self.ax.set_xlabel('X coordinate (m)')
            self.ax.set_ylabel('Y coordinate (m)')
            self.ax.set_title('Estimation in ENU frame')
            self.ax.grid(True)
            self.ax.set_xlim(self.xlim)
            self.ax.set_ylim(self.ylim)

            if (self.plot_defined_trace):
                x_trace = [p[0] for p in self.trace]
                y_trace = [p[1] for p in self.trace]
                self.ax.plot(x_trace, y_trace, '-o', 
                         label='Aimed path', markersize=5)
                self.ax.legend(loc=4)

            rospy.logdebug("In total {} topics.".format(serial))
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

        self.ax.plot(odom_x, odom_y, color=color, label=source)
        self.ax.axis('scaled')
        self.ax.legend(loc=4)
        # self..xlabel('X coordinate')
        # self..ylabel('Y coordinate')
        # self.title('Estimation in ENU frame')
        self.ax.grid(True)

    def plot_2D(self, x, y, color, source, serial):
        """Plot x and y coordinates."""
        self.ax.plot(x, y, color=color, label=source, 
                marker=".", markersize=self.point_size)
        if not self.legend_added[serial]:
            self.ax.legend(loc=4)
            self.legend_added[serial] = True
        rospy.logdebug("For {} to plot {}, {}.".format(source, x, y))

    def plot_3D(self, x, y, z, color, source, serial):
        """Plot xyz coordinates."""
        self.ax.plot(x, y, z, color=color, label=source, 
                marker=".", markersize=self.point_size)
        if not self.legend_added[serial]:
            self.ax.legend(loc=4)
            self.legend_added[serial] = True
        rospy.logdebug("For {} to plot {}, {}, {}.".format(source, x, y, z))

    def update_length_travelled(self, x, y):
        delta = sqrt((x-self.last_x)**2 + (y - self.last_y)**2)
        self.last_x = x
        self.last_y = y
        self.length_travelled += delta

    def plot_covariance(self, x, y, x_var, y_var, yaw_var):
        self.update_length_travelled(x, y)
        if self.plot_xy_covariance:
            self.ax_covar1.scatter(self.length_travelled, x_var, 
                                color='plum', label='x variance', 
                                marker=".")
            self.ax_covar1.scatter(self.length_travelled, y_var, 
                                color='tomato', label='y variance', 
                                marker=".")
            if not self.legend_added_xy_var:
                self.legend_added_xy_var = True
                self.ax_covar1.legend(loc=2)

        if self.plot_yaw_covariance:
            self.ax_covar2.scatter(self.length_travelled, yaw_var, 
                                color='salmon', label='yaw variance', marker=".")
            if not self.legend_added_yaw_var:
                self.legend_added_yaw_var = True
                self.ax_covar2.legend(loc=2)

    def plot_stdev(self, x, y, x_var, y_var, yaw_var):
        self.update_length_travelled(x, y)
        x_stdev = sqrt(x_var)
        y_stdev = sqrt(y_var)
        yaw_stdev = sqrt(yaw_var)
        if self.plot_xy_covariance:
            self.ax_covar1.scatter(self.length_travelled, x_stdev, 
                                color='plum', label='x stdev', 
                                marker=".")
            self.ax_covar1.scatter(self.length_travelled, y_stdev, 
                                color='tomato', label='y stdev', 
                                marker=".")
            if not self.legend_added_xy_var:
                self.legend_added_xy_var = True
                self.ax_covar1.legend(loc=2)

        if self.plot_yaw_covariance:
            self.ax_covar2.scatter(self.length_travelled, yaw_stdev, 
                                color='salmon', label='yaw stdev', marker=".")
            if not self.legend_added_yaw_var:
                self.legend_added_yaw_var = True
                self.ax_covar2.legend(loc=2)

    def odom_callback(self, msg, args):
        serial = args[0]
        color = args[1]
        source = args[2]
        angle = args[3]

        self.counter[serial] += 1
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        x_var = msg.pose.covariance[0]
        y_var = msg.pose.covariance[7]
        yaw_var = msg.pose.covariance[35]

        if(angle != 0.0):
            x, y = self.rotate_xypair(x, y, angle)
        if (self.counter[serial] % self.plot_per_points == 0):
            if (self.plot_3D_trace):
                self.plot_3D(x, y, z, color, source, serial)
            else:
                self.plot_2D(x, y, color, source, serial)
            if self.plot_total_num > 1:
                if self.plot_covariance_in_stdev:
                    self.plot_stdev(x, y, x_var, y_var, yaw_var)
                else:
                    self.plot_covariance(x, y, x_var, y_var, yaw_var)

    def pose_callback(self, msg, args):

        serial = args[0]
        color = args[1]
        source = args[2]
        angle = args[3]

        self.counter[serial] += 1
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        x_var = msg.pose.covariance[0]
        y_var = msg.pose.covariance[7]
        yaw_var = msg.pose.covariance[35]

        if(angle != 0.0):
            x, y = self.rotate_xypair(x, y, angle)
        if (self.counter[serial] % self.plot_per_points == 0):
            if (self.plot_3D_trace):
                self.plot_3D(x, y, z, color, source, serial)
            else:
                self.plot_2D(x, y, color, source, serial)
                if self.plot_covariance_in_stdev:
                    self.plot_stdev(x, y, x_var, y_var, yaw_var)
                else:
                    self.plot_covariance(x, y, x_var, y_var, yaw_var)

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
        if (self.plot_3D_trace):
            self.plot_3D(x, y, z, color, source, serial)
        elif (self.counter[serial] % self.plot_per_points == 0):
            self.plot_2D(x, y, color, source, serial)


if __name__ == '__main__':
    BagPlot()