#! /usr/bin/python3

'''
 Plot data stored in ROS bag with matplotlib for report's use.
 
 Procedure:
   1. Call rosbag API to extract the data. 
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


class BagPlot():

    def __init__(self):

        #  Bag location and read.
        home = expanduser("~")
        dir_name = home + "/bags/lawn/"
        filename = "lawn_one_lap_20201122.bag"
        self.bag = rosbag.Bag(dir_name + filename)

        # Configuration parameters.
        # Make sure the topic inclued are of type nav_msgs/Odometry.
        # May put in lists that will be ziped latter.
        # topic_list = ["/odom", "/odometry/filtered"]
        topic_list = ["/odom", "/odom"]
        color_list = ['r', 'g']
        angle_list = [75.0/180.0*np.pi, 0.0]
        source_list = ["Encoder", "Encoder not rotated"]

        self.do_rotate_map = True

        for topic, color, angle, source in zip(topic_list, color_list, 
                                       angle_list, source_list): 
            self.topic_plot(topic, color, angle, source)

        plt.show()
        self.bag.close()


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

        plt.plot(odom_x, odom_y, color = color, label = source)
        plt.axis('scaled')
        plt.legend()
        plt.xlabel('X coordinate')
        plt.ylabel('Y coordinate')
        plt.title('Estimation in ENU frame')
        plt.grid(True)


if __name__ == '__main__':
    # rospy.init_node('bag_plot')
    BagPlot()
    # rospy.spin()