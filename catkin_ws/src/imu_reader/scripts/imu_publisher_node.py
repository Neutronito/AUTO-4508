from Phidget22.Phidget import *
from Phidget22.Devices.Spatial import *
import time

#Declare any event handlers here. These will be called every time the associated event occurs.

def onAlgorithmData(self, quaternion, timestamp):
	print("Timestamp: " + str(timestamp))

	eulerAngles = self.getEulerAngles()
	print("EulerAngles: ")
	print("\tpitch: " + str(eulerAngles.pitch))
	print("\troll: " + str(eulerAngles.roll))
	print("\theading: " + str(eulerAngles.heading))

	quaternion = self.getQuaternion()
	print("Quaternion: ")
	print("\tx: " + str(quaternion.x))
	print("\ty: " + str(quaternion.y))
	print("\tz: " + str(quaternion.z))
	print("\tw: " + str(quaternion.w))
	print("----------")

def main():
	#Create your Phidget channels
	spatial0 = Spatial()

	#Set addressing parameters to specify which channel to open (if any)
	spatial0.setDeviceSerialNumber(373542)

	#Assign any event handlers you need before calling open so that no events are missed.
	spatial0.setOnAlgorithmDataHandler(onAlgorithmData)

	#Open your Phidgets and wait for attachment
	spatial0.openWaitForAttachment(5000)

	#Do stuff with your Phidgets here or in your event handlers.
	spatial0.setHeatingEnabled(True)

	try:
		input("Press Enter to Stop\n")
	except (Exception, KeyboardInterrupt):
		pass

	#Close your Phidgets once the program is done.
	spatial0.close()

main()