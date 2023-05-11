#include "ros/ros.h"
#include <stdlib.h>
#include "waypoint_driver/gps_points.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/NavSatFix.h"

float requestLatitude;
float requestLongitude;
bool currentlyDriving;

float curHeading;
bool recieved_heading = false;

float curLatitude;
float curLongitude;
bool recievedCoords = false;

// 1 topics, 2 service

// First service is used to initialize way point driving, if driving is already underway false is returned as an error.
// Second service is used to pause driving or terminate current waypoint
// Third topic posts when a task is complete

// Gets the imu heading and updates global flags and variables
void imu_heading_callback(const std_msgs::Float64::ConstPtr& msg) {
    if (currentlyDriving) {
		curHeading = msg->data;
		recieved_heading = true;
	}
}

// Gets the gps coordinates and updates global flags and variables
void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
	if (currentlyDriving) {
		curLatitude = msg->latitude;
		curLongitude = msg->longitude;
		recievedCoords = true;
	}
}


bool drive_waypoint(waypoint_driver::gps_points::Request  &req, waypoint_driver::gps_points::Response &res) {
	// Parse recieved data
	double latitude = req.latitude;
	double longitude = req.latitude;
	ROS_INFO("Recieved request of: lat=%f, long=%f", latitude, longitude);
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
	
	// Now commence the loop, our loop rate will be 10Hz
	ros::Rate loop_rate(10);

	while (ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	}


	return 0;
}