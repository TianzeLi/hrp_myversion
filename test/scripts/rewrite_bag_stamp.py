#! /usr/bin/python3

"""
 Rewrite the timestamp of certain topics in the rosbag. 
 In particular, the timestamp can be unified when recorded on distributed
 devices that offer different time stamps. 

 t is the time the the message is received, while header.stamp can contain the 
 time when the message is generated.

"""

import rosbag 
import rospy
from os.path import expanduser
import math
import yaml
import sys
import subprocess
import time


def status(length, percent):
    sys.stdout.write('\x1B[2K') # Erase entire current line
    sys.stdout.write('\x1B[0E') # Move to the beginning of the current line
    progress = "Progress: ["
    for i in range(0, length):
        if i < length * percent:
            progress += '='
        else:
            progress += ' '
    progress += "] " + str(round(percent * 100.0, 2)) + "%"
    sys.stdout.write(progress)
    sys.stdout.flush()


#  Bag location and read.
home = expanduser("~")
dir_name = home + "/bags/lawn/"
# filename = "lawn_one_lap_20201122"
filename = "lawn_three_laps_20201122"
orig_bag = rosbag.Bag(dir_name + filename + ".bag")
restamped_path = dir_name + filename + "_restamped.bag"
# Threshold that if larger, consider as the lagged time stamp
max_offset = 5.0


info_dict = yaml.load(subprocess.Popen(['rosbag', 'info', '--yaml', 
                                        (dir_name + filename + ".bag")], 
                                        stdout=subprocess.PIPE).communicate()[0],
                                        Loader=yaml.FullLoader)
                                        
duration = info_dict['duration']
start_time = info_dict['start']


with rosbag.Bag(restamped_path, 'w') as outbag:

    last_time = time.clock()
    for topic, msg, t in orig_bag.read_messages():
        if (time.clock() - last_time > .1) and (t.to_sec() > start_time):
            percent = (t.to_sec() - start_time) / duration
            status(40, percent)
            last_time = time.clock()

        if topic == "/tf" and msg.transforms:
          # Writing transforms to bag file 1 second ahead of time to ensure availability
            for transform in msg.transforms:
                if t.to_sec() - transform.header.stamp.to_sec() > 100.0:
                    transform.header.stamp = t
            outbag.write(topic, msg, t)
        elif msg._has_header:
            if t.to_sec() - msg.header.stamp.to_sec() > 100.0:
                msg.header.stamp = t
            outbag.write(topic, msg, t)
        else:
            outbag.write(topic, msg, t)

status(40, 1)
print("\nDone.")