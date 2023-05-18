#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int16.h"
#include <math.h>

ros::Publisher lidar_pub;

void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    std_msgs::Int16 lidar_response;
    lidar_response.data = 0;

    float thres = 3.f;          //threshold set to 1m
    int size = msg->ranges.size();

    //Start points for the 3 thirds of the lidar
    int mid = size/2;
    float angle_increment = 180*(msg->angle_increment)/M_PI; //Normally 0.33 degrees
    int left_start = mid - abs(30/angle_increment);
    int left_end = mid - abs(10/angle_increment);
    int right_start = mid + abs(10/angle_increment);
    int right_end = mid + abs(30/angle_increment);
    int count = 0;

    //Check the left third
    for(int i = left_start; i < left_end; i++)
    {
        if(msg->ranges[i] < thres && msg->ranges[i] > 0.05)
        {
            lidar_response.data = 1;
            break;
        }
    }

    //Check the right third
    for(int i = right_start; i < right_end; i++)
    {
        if(msg->ranges[i] < thres && msg->ranges[i] > 0.05)
        {
            lidar_response.data = -1;
            break;
        }
    }

    //Check the middle third
    for(int i = left_end; i < right_start; i++)
    {
        if(msg->ranges[i] < thres && msg->ranges[i] > 0.05)
        {
            lidar_response.data = 2;
            break;
        }
    }

    // Nowcheck yomama
    
    // 1 obstacle on right, -1 obstacle on left, 2 obstacle in front, 0 no obstacle wooohoooo
    lidar_pub.publish(lidar_response);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_response_node");

    ros::NodeHandle nh;


    ros::Subscriber lidar_get = nh.subscribe("sick_tim_7xx/scan", 10, lidarCallback);
    lidar_pub = nh.advertise<std_msgs::Int16>("lidar_response", 10);

    ros::spin();

    return 0;
}