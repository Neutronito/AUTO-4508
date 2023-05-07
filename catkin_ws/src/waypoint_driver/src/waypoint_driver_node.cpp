#include "ros/ros.h"
#include "waypoint_driver/gps_points.h"

bool drive_waypoint(waypoint_driver::gps_points::Request  &req, waypoint_driver::gps_points::Response &res) {
  double latitude = req.latitude;
  double longitude = req.latitude;
  ROS_INFO("Recieved request of: lat=%f, long=%f", latitude, longitude);
  bool returnval = false;
  ROS_INFO("sending back response: [%d]", returnval);
  res.reached_status = returnval;
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "waypoint_driver");
  ros::NodeHandle waypoint_handle;

  ros::ServiceServer service = waypoint_handle.advertiseService("gps_points", drive_waypoint);
  ROS_INFO("Ready to add two ints.");
  ros::spin();

  return 0;
}