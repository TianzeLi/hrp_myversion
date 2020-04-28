#!/usr/bin/env python
"""

	Get the latitude and longitude from the GNSS and publish the postion 
	in the type of geometry_msgs/PoseWithCovarianceStamped.

"""

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped 
from Phidget22.PhidgetException import *
from Phidget22.Phidget import *
from Phidget22.Devices.GPS import *
import time



# Define the initial position and standard variance of measurement of the GNSS  
# Unit: meter
############################################################################################
# TODO: introduce offset, could through the difference of first measurements and true value?
############################################################################################
INITIAL_LATITUDE = 59.40455 
INITIAL_LONGITUDE = 17.94965
LATITUDE_STDEV = 5
LONGITUDE_STDEV = 5
DECI_DEGREE_TO_METER = 111320
MOVE_X = 0
MOVE_Y = 0

# Define how many measurements are averaged. 
# The default measuring rate is 10Hz. 
####################################
# TODO: introduce averaged measuring
####################################


pose_pub = rospy.Publisher("gnss/pose", PoseWithCovarianceStamped, queue_size=10)


#Declare any event handlers here. These will be called every time the associated event occurs.
def onPositionChange(self, latitude, longitude, altitude):
	print("Latitude: " + str(latitude))
	print("Longitude: " + str(longitude))
	print("Altitude: " + str(altitude))
	print("----------")

	# Conerting from decimal degree into meter and into the map frame.
	############################################################
	# TODO: take in the orientation difference of map and north.
	############################################################
	MOVE_X = (longitude - INITIAL_LONGITUDE)*DECI_DEGREE_TO_METER
	MOVE_Y = (latitude - INITIAL_LATITUDE)*DECI_DEGREE_TO_METER
	# MOVE_Z = 

	pose = PoseWithCovarianceStamped()
	pose.header.stamp = rospy.Time.now()
	pose.header.frame_id = "map"
	pose.pose.pose.position.x = MOVE_X
	pose.pose.pose.position.y = MOVE_Y
	pose.pose.covariance[0] = LATITUDE_STDEV
	pose.pose.covariance[7] = LONGITUDE_STDEV
	pose_pub.publish(pose)


# def onHeadingChange(self, heading, velocity):
# 	print("Heading: " + str(heading))
# 	print("Velocity: " + str(velocity))

# def onAttach(self):
# 	print("Attach!")

# def onDetach(self):
# 	print("Detach!")

def onError(self, code, description):
	print("Code: " + ErrorEventCode.getName(code))
	print("Description: " + str(description))
	print("----------")

def gnss_pub_node():
	rospy.init_node("gnss_pub", anonymous=True)

	# Fetch the parameters
	device_serial_number = rospy.get_param('~device_serial_number')

	#Create your Phidget channels
	gps0 = GPS()

	#Set addressing parameters to specify which channel to open
	gps0.setDeviceSerialNumber(device_serial_number)

	#Assign any event handlers you need before calling open so that no events are missed.
	gps0.setOnPositionChangeHandler(onPositionChange)
	# gps0.setOnHeadingChangeHandler(onHeadingChange)
	# gps0.setOnAttachHandler(onAttach)
	# gps0.setOnDetachHandler(onDetach)
	gps0.setOnErrorHandler(onError)

	#Open your Phidgets and wait for attachment
	gps0.open()

	while not rospy.is_shutdown():

		# Wait for .1 second.
		time.sleep(.1)

		#Close your Phidgets once the program is done.
		# gps0.close()


if __name__ == '__main__':
	try:
		gnss_pub_node()
	except rospy.ROSInterruptException:
		pass