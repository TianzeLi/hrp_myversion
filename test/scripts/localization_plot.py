#! /usr/bin/python3

import rospy
# import rosbag
import roslaunch
import rospkg
import matplotlib.pyplot as plt
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point


def launch():
	launch_args = []
	launch_files = []
	cli_args1 = ['am_test', 'bag_play.launch']
	cli_args2 = ['am_test', 'bag_ekf.launch']
	launch_args.append(cli_args1)
	launch_args.append(cli_args2)

	uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
	roslaunch.configure_logging(uuid)
	for arg in launch_args:
		file = roslaunch.rlutil.resolve_launch_arguments(arg)[0]
		print(file)
		launch_files.append(file)		
	print(launch_files)	
	parent = roslaunch.parent.ROSLaunchParent(uuid, launch_files)
	parent.start()

	try:
	  parent.spin()
	finally:
	  parent.shutdown()


if __name__ == "__main__":
	launch()

