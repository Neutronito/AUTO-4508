#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

void scanTopicCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidar_values_interpreter_node");


  ros::NodeHandle lidar_nodehandle;

  ros::Subscriber sub = lidar_nodehandle.subscribe("/scan", 10, scanTopicCallback);

  ros::spin();

  return 0;
}