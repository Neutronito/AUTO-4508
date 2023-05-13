#include "ros/ros.h"
#include <stdlib.h>
#include <math.h>
#include "waypoint_driver/gps_points.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/NavSatFix.h"
#include <geometry_msgs/Twist.h>
#include "waypoint_driver/finished_state.h"

#define GOALTOLERANCE 0.00001

#define MAXROTVEL 2
#define ROTSCALEFACTOR 0.0222	

float requestLatitude;
float requestLongitude;
bool currentlyDriving;

float curHeading;
bool receivedHeading = false;

float curLatitude;
float curLongitude;
bool receivedCoords = false;

float linearVelocity = 0;
float angularVelocity = 0;

// 1 topics, 2 service

// First service is used to initialize way point driving, if driving is already underway false is returned as an error.
// Second service is used to pause driving or terminate current waypoint
// Third topic posts when a task is complete

// Gets the imu heading and updates global flags and variables
void imu_heading_callback(const std_msgs::Float64::ConstPtr& msg) {
    if (currentlyDriving) {
		curHeading = msg->data;
		// map heading 0-360
		curHeading = curHeading + 360 % 360;
		receivedHeading = true;
	}
}

// Gets the gps coordinates and updates global flags and variables
void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
	if (currentlyDriving) {
		curLatitude = msg->latitude;
		curLongitude = msg->longitude;
		receivedCoords = true;
	}
}


bool drive_waypoint(waypoint_driver::gps_points::Request  &req, waypoint_driver::gps_points::Response &res) {
	// Parse received data
	double latitude = req.latitude;
	double longitude = req.longitude;
	ROS_INFO("received request of: lat=%f, long=%f", latitude, longitude);
	if (currentlyDriving) {
		ROS_INFO("I'm already driving, that waypoint request will be refused");
		res.approved_request = false;
		return true;
	} else {
		ROS_INFO("Coordinate request approved");
		
		// Set the flags
		requestLatitude = latitude;
		requestLongitude = longitude;
		currentlyDriving = true;
		
		res.approved_request = true;
		return true;		
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "waypoint_driver");
	ros::NodeHandle waypoint_handle;
	
	// start the way point driving service
	ros::ServiceServer service = waypoint_handle.advertiseService("gps_points", drive_waypoint);

	// Subscribe to imu
	ros::Subscriber imu_subscriber = waypoint_handle.subscribe("imu_heading", 10, imu_heading_callback);
	// Subscribe to gps
	ros::Subscriber gps_subscriber = waypoint_handle.subscribe("fix", 10, gps_callback);

	// Setup publishing to rosaria cmd_vel
	ros::Publisher velocity_publisher = waypoint_handle.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 10);

	// Set up publishing to finished state
	ros::Publisher finished_publisher = waypoint_handle.advertise<waypoint_driver::finished_state>("waypoint_driver/finished_state", 10);

	// Now commence the loop, our loop rate will be 10Hz
	ros::Rate loopRate(10);

	// This is our main loop
	while (ros::ok()){
		
		// Check if we are currently driving
		if (currentlyDriving) {

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
						currentlyDriving = false;
						
						ROS_INFO("Publishing information to finished state topic")
						waypoint_driver::finished_state msgToSend;
						msgToSend.current_latitude = curLatitude;
						msgToSend.current_longitude = curLongitude;
						msgToSend.reached_waypoint = true;
						finished_publisher.publish(msgToSend);
					}
				}

				// Find the heading we need to drive at
				float desiredHeading = atan2(cos(requestLatitude) * sin(requestLongitude - curLongitude), cos(curLatitude) * sin(requestLatitude) - sin(curLatitude) * cos(requestLatitude) * cos(requestLongitude - curLongitude));
				desiredHeading = desiredHeading * 180 / M_PI;
				
				// Map 0-360
				desiredHeading = desiredHeading + 360 % 360;

				// Find the two angles to turn
				float cw = desiredHeading - curHeading;
				cw = cw + 360 % 360;

				float ccw = 360 - cw;

				// Take the smallest angle
				float angleToDrive = cw;

				if (ccw < cw) {
					angleToDrive = 0 - ccw;
				}

				angleToDrive *= -1;

				// Now set the angular velocity based on our angle
				angularVelocity = angleToDrive * ROTSCALEFACTOR;

				if (angularVelocity < (0 - MAXROTVEL)) {
					angularVelocity = 0 - MAXROTVEL;
				} else if (angularVelocity > MAXROTVEL) {
					angularVelocity = MAXROTVEL;
				}
				ROS_INFO("Current heading is %f, desired heading is %f and rot vel is set to %f", curHeading, desiredHeading, angularVelocity);
				linearVelocity = 1;
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
