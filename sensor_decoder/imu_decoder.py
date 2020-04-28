from Phidget22.Phidget import *
from Phidget22.Devices.Accelerometer import *
import time

def onAccelerationChange(self, acceleration, timestamp):
	print("Acceleration: \t"+ str(acceleration[0])+ "  |  "+ str(acceleration[1])+ "  |  "+ str(acceleration[2]))
	print("Timestamp: " + str(timestamp))
	print("----------")

def main():
	accelerometer0 = Accelerometer()

	accelerometer0.setOnAccelerationChangeHandler(onAccelerationChange)

	accelerometer0.openWaitForAttachment(5000)

	try:
		input("Press Enter to Stop\n")
	except (Exception, KeyboardInterrupt):
		pass

	accelerometer0.close()

main()
