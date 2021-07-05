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


class BagPlot():

    def __init__(self):

        self.plot_from_bag = False
        self.plot_3D = False

        # Configuration parameters.
        # Make sure the topic inclued are of type nav_msgs/Odometry.
        # May put in lists that will be ziped latter.
        # topic_list = ["/odom", "/odometry/filtered"]
        topic_list = ["/odom", "/odom"]
        type_list = ["Odometry", "Odometry"]
        color_list = ['r', 'g']
        angle_list = [75.0/180.0*np.pi, 0.0]
        source_list = ["Encoder", "Encoder not rotated"]

        self.do_rotate_map = True


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
            plt.ion()
            # fig = plt.figure() 
            plt.legend()
            plt.xlabel('X coordinate')
            plt.ylabel('Y coordinate')
            plt.title('Estimation in ENU frame')
            plt.grid(True)

            serial = 0
            self.counter = 0
            self.points = []
            for topic, topic_type, color, angle, source in zip(
                    topic_list, type_list, color_list, angle_list, source_list):
                self.points.append([])
                rospy.logdebug("We have topic {} of type {} in color {}."
                        .format(topic, topic_type, color)) 
                if (topic_type == "Odometry"): 
                    rospy.Subscriber(topic, Odometry, self.odom_callback,
                                    (serial, color, source, angle))
                if (topic_type == "PoseWithCovarianceStamped"): 
                    rospy.Subscriber(topic, PoseWithCovarianceStamped, self.pose_callback, 
                                    (serial, color, source, angle))
                serial += 1

            rospy.logdebug("In total {} topics.".format(serial)) 
            plt.show(block=True) 
            rospy.spin()


    def rotate_xypair(self, x, y, theta):
        x_rotated = x*np.cos(theta) - y*np.sin(theta)
        y_rotated = x*np.sin(theta) + y*np.cos(theta)
        return x_rotated, y_rotated


    def topic_plot(self, topic_name, color, angle, source): 

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


    def odom_callback(self, msg, args):

        serial = args[0]
        color = args[1]
        source = args[2]
        angle = args[3]

        rospy.logdebug("We have arguments {}, {}, {}."
                        .format(serial, color, source)) 

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        if(angle != 0.0):
            x, y = self.rotate_xypair(x, y, angle)
        if (self.plot_3D):
            self.points[serial].append([x, y, z])
        # elif (self.counter % 10 == 0):
            # self.counter += 1
        else:
            self.points[serial].append([x, y])
            plt.plot(x, y, '*', color = color, label = source)
            plt.axis('scaled')
            plt.draw()
            plt.pause(0.0001)
            rospy.logdebug("In the topic No.{} to plot {}, {}."
                            .format(serial, x, y)) 

    def pose_callback(self, msg, args):

        serial = args[0]
        color = args[1]
        source = args[2]
        angle = args[3]

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        if(angle != 0.0):
            x, y = self.rotate_xypair(x, y, angle)
        if (self.plot_3D):
            self.points[serial].append([x, y, z])
        else:
            self.points[serial].append([x, y])
            plt.plot(x, y, color=color, label=source)
            plt.axis('scaled')
            plt.draw()
            plt.pause(0.0001)
            rospy.logdebug("In the topic No.{} to plot {}, {}."
                            .format(serial, x, y)) 


if __name__ == '__main__':
    BagPlot()