#!/usr/bin/env python
"""
	Get the latitude and longitude from the GNSS and publish: 
	1. latitude, longitude and altitude in sensor_msgs/NavSatFix
	2. relative postion to the initial point in ENU 
	   in geometry_msgs/PoseWithCovarianceStamped.

	Note that now I do not have the access to the device and the program
	directly manipulates the GNSS API, I cannot test after modifying, so I 
	will leave it in its rugged shape:)

"""

import rospy
import time
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped, Vector3
from sensor_msgs.msg import NavSatFix 
from Phidget22.PhidgetException import *
from Phidget22.Phidget import *
from Phidget22.Devices.GPS import *


# Define the initial position and standard variance of measurement of the GNSS  
# Unit: meter
DECI_DEGREE_TO_METER = 111320
INITIAL_LATITUDE = 59.40455 
INITIAL_LONGITUDE = 17.94965
LATITUDE_STDEV = 5
LONGITUDE_STDEV = 5

# For warming up.
device_serial_number = 0
ENABLE_PUBLISH = 0
warming_time = 30
pub_frequency = 10
CONTROL_STDEV = 5
warming_queue = []

# ROS elements.
global_pose_pub = rospy.Publisher("fix", NavSatFix, queue_size=10)
local_pose_pub = rospy.Publisher("pose", PoseWithCovarianceStamped, queue_size=10)
global_hv_pub = rospy.Publisher("hv", Vector3, queue_size=10)


def onPositionChange(self, latitude, longitude, altitude):

	global INITIAL_LATITUDE
	global INITIAL_LONGITUDE
	global LATITUDE_STDEV
	global LONGITUDE_STDEV
	global DECI_DEGREE_TO_METER

	global device_serial_number
	global ENABLE_PUBLISH
	global warming_time
	global pub_frequency
	global CONTROL_STDEV
	global warming_queue

	fix = NavSatFix()
	fix.header.stamp = rospy.Time.now()
	fix.header.frame_id = "world"
	fix.latitude = latitude
	fix.longitude = longitude
	fix.altitude = altitude
	global_pose_pub.publish(fix)

	if (ENABLE_PUBLISH == 0):
		# Store in the warming_queue and compute 
		queue_len = len(warming_queue)
		warming_queue.append((longitude, latitude))

		if (queue_len >= warming_time*pub_frequency):
			warming_queue.pop(0)

			long_sum = 0.0
			lati_sum = 0.0
			long_var = 0.0
			lati_var = 0.0

			for i in range(queue_len):
				long_sum += warming_queue[i][0]
				lati_sum += warming_queue[i][1]
				long_var += warming_queue[i][0]**2
				lati_var += warming_queue[i][1]**2
			# rospy.logdebug("Current longitude variance: [%f]", long_var )
			# rospy.logdebug("Current latitude variance: [%f]", lati_var )

			long_average = long_sum/queue_len
			lati_average = lati_sum/queue_len
			long_var = (long_var/queue_len - long_average**2)*DECI_DEGREE_TO_METER**2
			lati_var = (lati_var/queue_len - lati_average**2)*DECI_DEGREE_TO_METER**2
			
			rospy.logdebug("Current average longitude: [%.6f]", long_average )
			rospy.logdebug("Current average latitude: [%.6f]", lati_average )
			# rospy.logdebug("Current longitude variance: [%f]", long_var )
			# rospy.logdebug("Current latitude variance: [%f]", lati_var )
			# rospy.logdebug("Initial longitude stdev: [%f]", np.sqrt(long_var + lati_var)*DECI_DEGREE_TO_METER )

			if (np.sqrt(long_var + lati_var) < CONTROL_STDEV):
				ENABLE_PUBLISH = 1
				LONGITUDE_STDEV = np.sqrt(long_var)
				LATITUDE_STDEV = np.sqrt(lati_var)
				INITIAL_LONGITUDE = long_average
				INITIAL_LATITUDE = lati_average

				rospy.loginfo("GNSS [%s] stdev within control value.", device_serial_number )
				rospy.loginfo("Initial longitude: [%.6f]", long_average )
				rospy.loginfo("Initial latitude: [%.6f]", lati_average )
				rospy.loginfo("Initial longitude stdev: [%f]", LONGITUDE_STDEV )
				rospy.loginfo("Initial latitude stdev: [%f]", LATITUDE_STDEV )
			else: 
				# rospy.logdebug("Current average longitude: [%f]", long_average )
				# rospy.logdebug("Current average latitude: [%f]", lati_average )
				rospy.logdebug("Current position stdev in meter: [%f]", np.sqrt(long_var + lati_var) )



	if(ENABLE_PUBLISH == 1):
		# Converting from decimal degree into meter and into the map frame.
		#############################################################
		# TODO: take in the orientation difference of map and north??
		#############################################################
		MOVE_X = (longitude - INITIAL_LONGITUDE)*DECI_DEGREE_TO_METER
		MOVE_Y = (latitude - INITIAL_LATITUDE)*DECI_DEGREE_TO_METER
		# MOVE_Z = 
		pose = PoseWithCovarianceStamped()
		pose.header.stamp = rospy.Time.now()
		pose.header.frame_id = "map"
		pose.pose.pose.position.x = MOVE_X
		pose.pose.pose.position.y = MOVE_Y
		# When processing, double the covariance
		pose.pose.covariance[0] = LATITUDE_STDEV*2
		pose.pose.covariance[7] = LONGITUDE_STDEV*2
		local_pose_pub.publish(pose)

def onHeadingChange(self, heading, velocity):
	hv = Vector3()
	hv.x = heading
	hv.y = velocity
	global_hv_pub.publish(hv)

def onError(self, code, description):
	rospy.logerr("Code: %s", ErrorEventCode.getName(code))
	rospy.logerr("Description: %s", str(description))
	rospy.logerr("----------")

def gnss_pub_node():
	rospy.init_node("gnss_pub", anonymous=True,  log_level=rospy.DEBUG)

	# Fetch the parameters
	device_serial_number = rospy.get_param('~device_serial_number')
	warming_time = rospy.get_param('~warming_time')
	pub_frequency = rospy.get_param('~pub_frequency')
	CONTROL_STDEV = rospy.get_param('~control_stdev')

	time_sleep = 1.0/pub_frequency

	#Create Phidget channels
	gps0 = GPS()
	#Set addressing parameters to specify which channel to open
	gps0.setDeviceSerialNumber(device_serial_number)
	#Assign any event handlers you need before calling open so that no events are missed.
	gps0.setOnPositionChangeHandler(onPositionChange)
	gps0.setOnHeadingChangeHandler(onHeadingChange)
	# gps0.setOnAttachHandler(onAttach)
	# gps0.setOnDetachHandler(onDetach)
	gps0.setOnErrorHandler(onError)

	#Open your Phidgets and wait for attachment
	gps0.open()
	while not rospy.is_shutdown():
		# Wait for 1/f second.
		time.sleep(time_sleep)

	#Close your Phidgets once the program is done.
	# gps0.close()

if __name__ == '__main__':
	try:
		gnss_pub_node()
	except rospy.ROSInterruptException:
		pass