from Phidget22.PhidgetException import *
from Phidget22.Phidget import *
from Phidget22.Devices.Accelerometer import *
from Phidget22.Devices.Gyroscope import *
from Phidget22.Devices.Magnetometer import *
import traceback
import time

#Declare any event handlers here. These will be called every time the associated event occurs.

def onAccelerationChange(self, acceleration, timestamp):
	print("Acceleration: \t"+ str(acceleration[0])+ "  |  "+ str(acceleration[1])+ "  |  "+ str(acceleration[2]))
	print("Timestamp: " + str(timestamp))
	print("----------")

def onAngularRateUpdate(self, angularRate, timestamp):
	print("AngularRate: \t"+ str(angularRate[0])+ "  |  "+ str(angularRate[1])+ "  |  "+ str(angularRate[2]))
	print("Timestamp: " + str(timestamp))
	print("----------")

def onMagneticFieldChange(self, magneticField, timestamp):
	print("MagneticField: \t"+ str(magneticField[0])+ "  |  "+ str(magneticField[1])+ "  |  "+ str(magneticField[2]))
	print("Timestamp: " + str(timestamp))
	print("----------")

def main():
	try:
		#Create your Phidget channels
		accelerometer0 = Accelerometer()
		gyroscope0 = Gyroscope()
		magnetometer0 = Magnetometer()

		#Set addressing parameters to specify which channel to open (if any)

		#Assign any event handlers you need before calling open so that no events are missed.
		accelerometer0.setOnAccelerationChangeHandler(onAccelerationChange)
		gyroscope0.setOnAngularRateUpdateHandler(onAngularRateUpdate)
		magnetometer0.setOnMagneticFieldChangeHandler(onMagneticFieldChange)

		#Open your Phidgets and wait for attachment
		accelerometer0.openWaitForAttachment(5000)
		gyroscope0.openWaitForAttachment(5000)
		magnetometer0.openWaitForAttachment(5000)

		#Do stuff with your Phidgets here or in your event handlers.

		time.sleep(5)

		#Close your Phidgets once the program is done.
		accelerometer0.close()
		gyroscope0.close()
		magnetometer0.close()

	except PhidgetException as ex:
		#We will catch Phidget Exceptions here, and print the error informaiton.
		traceback.print_exc()
		print("")
		print("PhidgetException " + str(ex.code) + " (" + ex.description + "): " + ex.details)


main()
