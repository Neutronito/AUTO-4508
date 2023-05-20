import rospy
from std_msgs.msg import Float64
from Phidget22.Phidget import *
from Phidget22.Devices.Spatial import *
import time

imu_publisher_handle = None
spatial_channel = None

#Declare any event handlers here. These will be called every time the associated event occurs.

def onAlgorithmData(self, quaternion, timestamp):
	global imu_publisher_handle
	
	eulerAngles = self.getEulerAngles()
	rospy.loginfo("Broadcasting a heading of " + str(eulerAngles.heading))
	imu_publisher_handle.publish(eulerAngles.heading)
	


def main():
	global imu_publisher_handle
	global spatial_channel

	# Now start the publisher
	imu_publisher_handle = rospy.Publisher('imu_heading', Float64, queue_size=10)
	rospy.init_node('imu_publisher', anonymous=True)
	
	#Create your Phidget channels
	spatial_channel = Spatial()

	#Set addressing parameters to specify which channel to open (if any)
	spatial_channel.setDeviceSerialNumber(373407)

	#Assign any event handlers you need before calling open so that no events are missed
	spatial_channel.setOnAlgorithmDataHandler(onAlgorithmData)

	#Open your Phidgets and wait for attachment
	spatial_channel.openWaitForAttachment(5000)

	#Turn on heating
	spatial_channel.setHeatingEnabled(True)

	# Attach the shutdown handler
	rospy.on_shutdown(shutdown_handler)
    
	rospy.loginfo("IMU Publisher node has successfully setup the IMU")

	rospy.spin()

def shutdown_handler():
	global spatial_channel

	# Close the phidget channel
	rospy.loginfo("Node kill recieved, shutting down phidget channel")
	spatial_channel.close()

main()