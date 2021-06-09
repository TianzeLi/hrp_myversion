#! /usr/bin/python3

'''
 Plot data stored in ROS bag with matplotlib for report's use.
 
 Procedure:
   1. Call rosbag API to extract the data. 
   2. Plot with the matplotlib.

 Elements in the figure for publish:
   1. Axis label.
   2. Title.
   3. Grid on.

 Note: 
   1. Python3 does not support LZ4, then we get the following warning: 
      Failed to load Python extension for LZ4 support.
   2. 
   
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
        topic_list = ["/odom"]
        color_list = ["r"]
        angle_list = [75.0/180.0*np.pi]

        self.do_rotate_map = True
        # self.rotate_theta = 75.0/180.0*np.pi

        for topic, color, angle in zip(topic_list, color_list, angle_list): 
            self.topic_plot(topic, color, angle)

        plt.show()
        self.bag.close()


    def rotate_xypair(self, x, y, theta):
        x_rotated = x*np.cos(theta) - y*np.sin(theta)
        y_rotated = x*np.sin(theta) + y*np.cos(theta)
        return x_rotated, y_rotated


    def topic_plot(self, topic_name, color, angle): 

        odom_x = []
        odom_y = []
        for topic, msg, t in self.bag.read_messages(topics=topic_name):
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            if(self.do_rotate_map):
                x, y = self.rotate_xypair(x, y, angle)
            odom_x.append(x)
            odom_y.append(y)

        plt.plot(odom_x, odom_y, color = color, label = "Encoder")
        plt.axis('scaled')
        plt.legend()
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Odom xy')
        plt.grid(True)


if __name__ == '__main__':
    # rospy.init_node('boundary_detect')
    BagPlot()
    # rospy.spin()



# plt.figure(fig_num)
# fig_num += 1
# plt.subplot(2, 1, 2)
# for i, col in enumerate(color_space):
# 	hist_hsv = cv2.calcHist([in_sample_hsv], [i], None, [256], [0, 256])
# 	plt.plot(hist_hsv, color = col, label = HSV[i])
# 	plt.xlim([0,256])
# plt.legend()
# plt.xlabel('Value')
# plt.ylabel('Count')
# plt.title('Sampled lawn pixels distribution in HSV space')
# plt.grid(True)

# plt.subplot(2, 1, 1)
# for i, col in enumerate(color_space):
# 	hist_hsv = cv2.calcHist([out_sample_hsv], [i], None, [256], [0, 256])
# 	plt.plot(hist_hsv, color = col, label = HSV[i])
# 	plt.xlim([0,256])
# plt.legend()
# plt.xlabel('Value')
# plt.ylabel('Count')
# plt.title('Sampled outliner pixels distribution in HSV space')
# plt.grid(True)

# plt.tight_layout()
# # plt.show()
