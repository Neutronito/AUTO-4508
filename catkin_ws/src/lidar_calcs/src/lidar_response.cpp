#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int16.h"

ros::Publisher lidar_pub;

void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    std_msgs::Int16 lidar_response;
    lidar_response.data = 0;

    //lidar range 0-25, the ranges maps them to 0-100, so in ranges 4 = 1m
    int thres = 8;          //threshold set to 2m
    int size = msg->ranges.size();
    
    //Start points for the 3 thirds of the lidar
    int mid = size/2;
    int left_start = mid - 30;
    int left_end = mid - 10;
    int right_start = mid + 10;
    int right_end = mid + 30;


    //Check the left third
    for(int i = left_start; i < left_end; i++)
    {
        if(msg->ranges[i] < thres)
        {
            lidar_response.data = -1;
            break;
        }
    }

    //Check the right third
    for(int i = right_start; i < right_end; i++)
    {
        if(msg->ranges[i] < thres)
        {
            lidar_response.data = 1;
            break;
        }
    }

    //Check the middle third
    for(int i = left_end; i < right_start; i++)
    {
        if(msg->ranges[i] < thres)
        {
            lidar_response.data = 2;
            break;
        }
    }

    lidar_pub.publish(lidar_response);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_response_node");

    ros::NodeHandle nh;


    ros::Subscriber lidar_get = nh.subscribe("scan", 10, lidarCallback);
    lidar_pub = nh.advertise<std_msgs::Int16>("lidar_response", 10);

    ros::spin();

    return 0;
}