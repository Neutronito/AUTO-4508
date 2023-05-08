#include "ros/ros.h"
#include "waypoint_driver/gps_points.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/NavSatFix.h"
#include <stdlib.h>

// THIS CODE IS STUPID
// Someone else tried to do this, this is how you would do it:
// https://answers.ros.org/question/339442/is-it-possible-to-have-a-subscriber-inside-the-service-server/

void gps_value_callback(const sensor_msgs::NavSatFix::ConstPtr& fix) {
	ROS_INFO("Recieved a navsat message, longitude is %f and latitude is %f", 180.0, 180.0);
}

void imu_value_callback(const std_msgs::Float64::ConstPtr& heading) {
	ROS_INFO("Recieved a heading, value is %f", 180.0);

}

bool drive_waypoint(waypoint_driver::gps_points::Request  &req, waypoint_driver::gps_points::Response &res) {
	// Parse recieved data
	double latitude = req.latitude;
	double longitude = req.latitude;
	ROS_INFO("Recieved request of: lat=%f, long=%f", latitude, longitude);
	bool returnval = false;

	// Start the gps_waypoint reader
	ros::Subscriber gps_subscriber = waypoint_handle.subscribe("fix", 10, gps_value_callback);

	// Start the imu reader
	ros::Subscriber imu_subscriber = waypoint_handle.subscribe("imu_heading", 10, imu_value_callback);

	while(true) {

	}

	// Send back the response
	ROS_INFO("sending back response: [%d]", returnval);
	res.reached_status = returnval;
	
	return true;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "waypoint_driver");
	ros::start();

	ros::NodeHandle waypoint_handle;

	// start the way point driving service
	// Boost is being used here to pass in the waypoint handle to the drive waypoin srv callback
	ros::ServiceServer service = waypoint_handle.advertiseService("gps_points",
		boost::bind(&drive_waypoint, boost::ref(waypoint_handle), _1));
	ROS_INFO("Ready to add two ints.");
	ros::spin();

	return 0;
}