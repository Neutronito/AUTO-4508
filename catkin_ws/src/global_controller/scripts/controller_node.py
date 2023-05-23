import rospy
import sys
import time
import csv
from enum import Enum
from sensor_msgs.msg import Joy
from global_controller.msg import controller_states
from pioneer_driver.srv import *
from drive_values.srv import *
from pioneer_driver.msg import *

STATE_PUBLISH_RATE = 60
TOGGLE_MINIMUM_INTERVAL = 1 # 1 second

# Represent joy topic button indexes as constants
CROSS, CIRCLE, TRIANGLE, SQUARE, L1, R1, L2, R2, SHARE, OPTIONS, CONNECT, JOY1, JOY2 = range(13)

# Define a Controller class to act as a master node
class TheFatController:

	# GPS Coordinates represented as a list of tuples
	coordinates = []

	# cone detection topic publisher
	cone_detection_publisher = rospy.Publisher

	# Defines if we have reached our target or not
	reached_target = False

	# Defines if the cone finder has finished or not
	finished_cone = False

	# Driving states read from joystick
	states = {"is_driving_automatically": False,
				"is_allowed_to_drive": False}

	# The constructor
	def __init__(self, waypoint_file):
		# Last recorded time of automatic/manual toggle
		# Time given in float seconds
		self.last_toggle_time = rospy.get_time() 

		self.read_file_args(waypoint_file)
		return

	# Read and process the file containing waypoints
	def read_file_args(self, waypoint_file):
		# Attempt to open the file
		rospy.loginfo("Attempting to open file \"" + waypoint_file + "\".")
		
		with open(waypoint_file, 'r') as csvfile:
			reader = csv.reader(csvfile)

			for row in reader:
				if len(row) == 2:  # Ensure there are two columns in each row
					latitude = float(row[0])
					longitude = float(row[1])
					self.coordinates.append((latitude, longitude))

		# Now print the coordinates
		rospy.loginfo("Read the following coordinates: ")
		for coord_tuple in self.coordinates:
			rospy.loginfo(coord_tuple)

		return

	def waypoint_finished_callback(self, data):
		self.reached_target = True

	def cone_finished_callback(self, data):
		self.finished_cone = True

	# Subscribe to the joystick topic
	def setup_subs_and_pubs(self):
		rospy.Subscriber("joy", Joy, self.update_states)
		self.cone_detection_publisher = rospy.Publisher('cone_driving_detection/action_requests', action_requests, queue_size=5)
		rospy.Subscriber('waypoint_driver/finished_state', finished_state, self.waypoint_finished_callback)
		rospy.Subscriber('cone_driving_detection/finished_state', object_detection_finished, self.cone_finished_callback)
		return

	# Update internal state based on joystick settings
	def update_states(self, joy):
		# Test
		# rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.buttons)

		# Deadman switch
		self.states["is_allowed_to_drive"] = (joy.buttons[R2] == 1)

		if(joy.buttons[SQUARE] == 1 and \
			rospy.get_time() - self.last_toggle_time > TOGGLE_MINIMUM_INTERVAL):
			self.states["is_driving_automatically"] = not self.states["is_driving_automatically"]
			self.last_toggle_time = rospy.get_time()

			if self.states["is_driving_automatically"]:
				rospy.loginfo("Robot has been toggled to AUTOMATIC driving mode")
				self.enable_remote_driving(False)
				self.waypoint_status_change_client(False, False)

				# Unpause cone
				msg = action_requests()
				msg.start_cone_detection = False
				msg.pause_cone_detection = False
				msg.terminate_current_detection = False
				self.cone_detection_publisher.publish(msg)

			else:
				rospy.loginfo("Robot has been toggled to MANUAL driving mode")
				self.enable_remote_driving(True)
				self.waypoint_status_change_client(True, False)

				# Pause cone
				msg = action_requests()
				msg.start_cone_detection = False
				msg.pause_cone_detection = True
				msg.terminate_current_detection = False
				self.cone_detection_publisher.publish(msg)


		return

	# def publish_states(self):
	# 	pub = rospy.Publisher("/global_controller/states", controller_states, queue_size=3)
	# 	msg = controller_states()

	# 	while not rospy.is_shutdown():
	# 		msg.is_driving_automatically = self.states["is_driving_automatically"]
	# 		msg.is_allowed_to_drive      = self.states["is_allowed_to_drive"]
	# 		# Defaults to True
	# 		msg.tiz_is_boosted = True
	# 		pub.publish(msg)

	# 		rospy.sleep(1/STATE_PUBLISH_RATE)
	# 	return                   


	# Feed way point commands to the way point driving node
	def feed_waypoints(self):     
		# # Loop through all the way points, feeding each one into the robot
		for coordinate_tuple in self.coordinates:
			
			# Get the next coordinate
			response = self.gps_points_client(coordinate_tuple[0], coordinate_tuple[1])

			if response is False:
				# Terminate the previous drive and send the new one again
				rospy.loginfo("False returned upon attempting to start a new waypoint drive")

			# Set to pause if manual driving
			if (not self.states["is_driving_automatically"]):
				self.waypoint_status_change_client(True, False)

			# Wait for the target to be reached, loop runs at a rate of 10Hz
			rate = rospy.Rate(10) # 10Hz

			self.reached_target = False

			while (self.reached_target is False):
				rate.sleep()

			# Now run the cone detection node
			msg = action_requests()
			msg.start_cone_detection = True
			msg.pause_cone_detection = False
			msg.terminate_current_detection = False
			self.cone_detection_publisher.publish(msg)
			rospy.loginfo("Requested cone finder to start finding cones")
			
			while (self.finished_cone is False):
				rate.sleep()

			# Now we can move onto the next waypoint
			self.finished_cone = False
			self.reached_target = False


	# Client for feeding gps coords
	def gps_points_client(self, latitude, longitude):
		rospy.loginfo("About to send gps coordinates on waypoint_driver/gps_points srv, which are {0} and {1}".format(latitude, longitude))
		rospy.loginfo("Blocking until waypoint_driver/gps_points service becomes available")
		rospy.wait_for_service('waypoint_driver/gps_points')
		try:
			gps_points_service = rospy.ServiceProxy('waypoint_driver/gps_points', gps_points)
			response = gps_points_service(latitude, longitude)
			rospy.loginfo("Success using waypoint_driver/gps_points service, returned {0}".format(response))
			return response
		except rospy.ServiceException as e:
			rospy.logerr("Error, service call failed")
			return False

	# Client for toggling gps drive
	def waypoint_status_change_client(self, pause_state, request_termination):
		rospy.loginfo("Blocking until waypoint_driver/status_change service becomes available")
		rospy.wait_for_service('waypoint_driver/status_change')
		try:
			waypoint_status_change_service = rospy.ServiceProxy('waypoint_driver/status_change', status_change)
			response = waypoint_status_change_service(pause_state, request_termination)
			rospy.loginfo("Success using waypoint_driver/status_change service, returned {0}".format(response))
			return response
		except rospy.ServiceException as e:
			rospy.logerr("Error, service call failed")
			return False

	# Client for toggling gps drive
	def enable_remote_driving(self, enabled_state):
		rospy.loginfo("Blocking until drive_values/enable_remote service becomes available")
		rospy.wait_for_service('drive_values/enable_remote')
		try:
			waypoint_status_change_service = rospy.ServiceProxy('drive_values/enable_remote', enable_remote)
			response = waypoint_status_change_service(enabled_state)
			rospy.loginfo("Success using drive_values/enabled_remote service, returned {0}".format(response))
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
	master.setup_subs_and_pubs()
	master.feed_waypoints()
	rospy.spin()

