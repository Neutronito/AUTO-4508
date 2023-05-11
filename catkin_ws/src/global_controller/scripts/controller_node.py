import rospy
import sys
import time
from enum import Enum
from sensor_msgs.msg import Joy
from global_controller.msg import controller_states
from waypoint_driver.srv import *

STATE_PUBLISH_RATE = 60

# Represent joy topic button indexes as constants
CROSS, CIRCLE, TRIANGLE, SQUARE, L1, R1, L2, R2, SHARE, OPTIONS, CONNECT, JOY1, JOY2 = range(13)

# Define a Controller class to act as a master node
class TheFatController:

    # GPS Coordinates represented as a list of tuples
	coordinates = []

    # Defines if we have reached our target or not
	reached_target = False

	states = {"is_driving_automatically": False,
              "is_allowed_to_drive": False}

	# The constructor
	def __init__(self, waypoint_file):
		self.read_file_args(waypoint_file)
		self.feed_waypoints()
		return

	# Read and process the file containing waypoints
	def read_file_args(self, waypoint_file):
		# Attempt to open the file
		rospy.loginfo("Attempting to open file \"" + waypoint_file + "\".")
		try:
			waypoint_file = open(waypoint_file, mode="r")
		except IOError:
			rospy.logerr("Error, unable to open file.")
			sys.exit()

		# Get all the lines
		lines = waypoint_file.readlines()

		# Put all the coordinates into an integer array
		for current_line in lines:
			split_line = current_line.split(" ")
			lat = float(split_line[0])
			lng = float(split_line[1])
			self.coordinates.append((lat, lng))

		# Now print the coordinates
		rospy.loginfo("Read the following coordinates:")
		for coord_tuple in self.coordinates:
			rospy.loginfo(coord_tuple)

		# Close the file
		waypoint_file.close()
		return

	# Subscribe to the joystick topic
	def listen_to_joystick(self):
		rospy.Subscriber("joy", Joy, self.update_states)
		return

	# Update internal state based on joystick settings
	def update_states(self, joy):
		# Test
		# rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.buttons)

		# Deadman switch
		self.states["is_allowed_to_drive"] = (joy.buttons[R2] == 1)

		return

	def publish_states(self):
		pub = rospy.Publisher("/global_controller/states", controller_states, queue_size=3)
		msg = controller_states()

		while not rospy.is_shutdown():
			msg.is_driving_automatically = self.states["is_driving_automatically"]
			msg.is_allowed_to_drive      = self.states["is_allowed_to_drive"]
			# Defaults to True
			msg.tiz_is_boosted = True
			pub.publish(msg)

			rospy.sleep(1/STATE_PUBLISH_RATE)
		return

	# Set t                                                                                                                                                                                      f.current_waypoint = input_waypoint

	# Feed way point commands to the way point driving node
	def feed_waypoints(self):     

		final_waypoint_index = len(self.coordinates)

		# # Loop through all the way points, feeding each one into the robot
		for coordinate_tuple in self.coordinates:
			
			# Get the next coordinate
			response = self.gps_points_client(coordinate_tuple[0], coordinate_tuple[1])

			if response is False:
				# Terminate the previous drive and send the new one again
				rospy.loginfo("False returned upon attempting to start a new waypoint drive")

			while (self.reached_target is False):
				time.sleep(0.1)


	# Client for feeding gps coords
	def gps_points_client(self, latitude, longitude) -> bool:
		rospy.loginfo("About to send gps coordinates on /gps_points srv, which are {0} and {1}".format(latitude, longitude))
		rospy.loginfo("Blocking until /gps_points service becomes available")
		rospy.wait_for_service('/gps_points')
		try:
			gps_points_service = rospy.ServiceProxy('gps_points', gps_points)
			response = gps_points_service(latitude, longitude)
			rospy.loginfo("Success using /gps_points service, returned {0}".format(response))
			return response
		except rospy.ServiceException as e:
			rospy.logerr("Error, service call failed")
			return False

if __name__ == '__main__':
    
    # Check we have the correct number of inputs
    if len(sys.argv) < 2:
        rospy.logerr("Error, expected at least 1 argument but recieved none.")
        sys.exit()

    # Create a Controller object, use error checking
    rospy.init_node('TheFatController', anonymous=False)

    # I'll be honest the class is not really necessary here
    master = TheFatController(sys.argv[1])
    master.listen_to_joystick()
    master.publish_states()
    rospy.spin()

