#include "ros/ros.h"
#include <stdlib.h>
#include <math.h>
#include "waypoint_driver/gps_points.h"
#include "waypoint_driver/status_change.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/NavSatFix.h"
#include <geometry_msgs/Twist.h>
#include "waypoint_driver/finished_state.h"

#define GOALTOLERANCE 0.00001

#define MAXROTVEL 2
#define ROTSCALEFACTOR 0.0222	

bool onAJob = false;
bool pausedDriving = false;
bool deadman_pressed = false;


float requestLatitude;
float requestLongitude;

float curHeading;
bool receivedHeading = false;

float curLatitude;
float curLongitude;
bool receivedCoords = false;

float linearVelocity = 0;
float angularVelocity = 0;

int lidar_response = 0;
bool lidar_controlling = false;

// 1 topics, 2 service

// First service is used to initialize way point driving, if driving is already underway false is returned as an error.
// Second service is used to pause driving or terminate current waypoint
// Third topic posts when a task is complete

// Gets the imu heading and updates global flags and variables
void imu_heading_callback(const std_msgs::Float64::ConstPtr& msg) {
    if (onAJob) {
		curHeading = msg->data;
		// map heading 0-360
		receivedHeading = true;
	}
}

// Gets the gps coordinates and updates global flags and variables
void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
	if (onAJob) {
		curLatitude = msg->latitude;
		curLongitude = msg->longitude;
		receivedCoords = true;
	}
}

// Execute lidar avoidance stuff
void lidar_callback(const std_msgs::Int16::ConstPtr& msg) {
	lidar_response = msg->data;

	// Lidar only takes control when an obstacle is detected
	lidar_controlling = (lidar_response != 0);
}

bool drive_waypoint(waypoint_driver::gps_points::Request  &req, waypoint_driver::gps_points::Response &res) {
	// Parse received data
	double latitude = req.latitude;
	double longitude = req.longitude;
	ROS_INFO("received request of: lat=%f, long=%f", latitude, longitude);
	if (onAJob) {
		ROS_INFO("I'm already driving, that waypoint request will be refused");
		res.approved_request = false;
		return true;
	} else {
		ROS_INFO("Coordinate request approved");
		
		// Set the flags
		requestLatitude = latitude;
		requestLongitude = longitude;
		onAJob = true;
		pausedDriving = false;
		
		res.approved_request = true;
		return true;		
	}
}

bool triggered_status_change(waypoint_driver::status_change::Request &req, waypoint_driver::status_change::Response &res) {
	
	// Deal with pause request
	pausedDriving = req.pause;
	res.pause_state = pausedDriving;

	// Deal with termination request
	if (req.terminate_current_drive == true) {
		onAJob = false;
	}
	res.currently_on_job = onAJob;

	return true;
}

void deadman_callback(const std_msgs::Bool::ConstPtr& msg) {
	deadman_pressed = msg->data;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "waypoint_driver");
	ros::NodeHandle waypoint_handle;
	
	// start the way point driving service
	ros::ServiceServer service = waypoint_handle.advertiseService("waypoint_driver/gps_points", drive_waypoint);

	// start the status change srv
	ros::ServiceServer status_service = waypoint_handle.advertiseService("waypoint_driver/status_change", triggered_status_change);

	// Subscribe to imu
	ros::Subscriber imu_subscriber = waypoint_handle.subscribe("imu_heading", 10, imu_heading_callback);
	// Subscribe to gps
	ros::Subscriber gps_subscriber = waypoint_handle.subscribe("fix", 10, gps_callback);
	// Subscribe to lidar
	ros::Subscriber lidar_subscriber = waypoint_handle.subscribe("lidar_response", 10, lidar_callback);

	// Setup publishing to rosaria cmd_vel
	ros::Publisher velocity_publisher = waypoint_handle.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 10);

	// Set up publishing to finished state
	ros::Publisher finished_publisher = waypoint_handle.advertise<waypoint_driver::finished_state>("waypoint_driver/finished_state", 10);

	// Setup subscription to deadman topic
	ros::Subscriber deadman_subscriber = waypoint_handle.subscribe("drive_values/deadman_state", 10, deadman_callback);

	// Now commence the loop, our loop rate will be 10Hz
	ros::Rate loopRate(10);

	// This is our main loop
	while (ros::ok()){
		
		// Check if we are currently driving
		if (onAJob && !pausedDriving && deadman_pressed) {

			// Check if we are obstacle avoiding
			if (lidar_controlling) {

				// Veer right, obstacle on the left 
				if (lidar_response == -1) {
					angularVelocity = -0.5;
					linearVelocity = 1;
				} 
				
				// Veer left, obstacle on right
				else if (lidar_response == 1) {
					angularVelocity = 0.5;
					linearVelocity = 1;
				}

				// Obstacle in front, I've chosen to turn right away instead of left (no specific reason)
				else if (lidar_response == 2) {
					linearVelocity = 0;
					angularVelocity = 1;
				} 
			}

			else {
				// Check if new data has come through
				if (receivedHeading) {
					receivedHeading = false;
					receivedCoords = false;

					// Check if we are at the goal
					if (curLatitude > requestLatitude - GOALTOLERANCE && curLatitude < requestLatitude + GOALTOLERANCE) {
						if (curLongitude > requestLongitude - GOALTOLERANCE && curLongitude < requestLongitude + GOALTOLERANCE) {
							ROS_INFO("We have reached our desired coordinates");
							linearVelocity = 0;
							angularVelocity = 0;
							onAJob = false;
							
							ROS_INFO("Publishing information to finished state topic");
							waypoint_driver::finished_state msgToSend;
							msgToSend.current_latitude = curLatitude;
							msgToSend.current_longitude = curLongitude;
							msgToSend.reached_waypoint = true;
							finished_publisher.publish(msgToSend);
						}
					}

					// Find the heading we need to drive at, arctan doesnt map positive clockwise but somehow this formula does work and will map heading correctly.
					float desiredHeadingRad = atan2(cos(requestLatitude) * sin(requestLongitude - curLongitude), cos(curLatitude) * sin(requestLatitude) - sin(curLatitude) * cos(requestLatitude) * cos(requestLongitude - curLongitude));
					int desiredHeading = (int)(desiredHeadingRad * 180.0 / M_PI);
					
					// Map 0-360
					desiredHeading = (desiredHeading + 360) % 360;

					int currentHeading = ((int)curHeading + 360) % 360;

					// Find the two angles to turn
					int cw = desiredHeading - currentHeading;
					cw = (cw + 360) % 360;

					int ccw = 360 - cw;

					// Take the smallest angle
					int angleToDrive = cw;

					if (ccw < cw) {
						angleToDrive = 0 - ccw;
					}

					angleToDrive *= -1;

					// Now set the angular velocity based on our angle
					angularVelocity = (float)angleToDrive * ROTSCALEFACTOR;

					if (angularVelocity < (0 - MAXROTVEL)) {
						angularVelocity = 0 - MAXROTVEL;
					} else if (angularVelocity > MAXROTVEL) {
						angularVelocity = MAXROTVEL;
					}
					ROS_INFO("Current heading is %d, desired heading is %d, angle to turn is %d and rot vel is set to %f", currentHeading, desiredHeading, angleToDrive, angularVelocity);
					
					// If we are turning fast, we should reduce our linear velocity to not overtorque the battery
					linearVelocity = 2 - abs(angularVelocity);
					
					// Don't allow the linear velocity to be greater than 1
					if (linearVelocity > 1) linearVelocity == 1;
				}
			}

			// Publish the speeds to rosaria cmd_vel
			geometry_msgs::Twist velTwist;
			velTwist.angular.z = angularVelocity;
			velTwist.linear.x = linearVelocity;
			velocity_publisher.publish(velTwist);

		}

		ros::spinOnce();
		loopRate.sleep();
	}


	return 0;
}
