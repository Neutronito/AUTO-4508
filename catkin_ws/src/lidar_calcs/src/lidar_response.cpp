#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int16.h"
#include <math.h>
#include "lidar_calcs/front_object_distance.h"

ros::Publisher lidar_pub;

float lidar_mid_values[50];
float lidar_front_size = 0;

void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    std_msgs::Int16 lidar_response;
    lidar_response.data = 0;

    float thres_long = 1.5f;          //threshold set to 3m
    float thres_short = 1.5f;         //threshold set to 1m
    int size = msg->ranges.size();

    //Start points for the 3 thirds of the lidar
    int mid = size/2;
    float angle_increment = 180*(msg->angle_increment)/M_PI; //Normally 0.33 degrees
    int left_start = mid - abs(30/angle_increment);
    int left_end = mid - abs(10/angle_increment);
    int right_start = mid + abs(10/angle_increment);
    int right_end = mid + abs(30/angle_increment);

    //Check the left third
    for(int i = left_start; i < left_end; i++)
    {
        //Long range avoidance
        if(msg->ranges[i] < thres_long && msg->ranges[i] > 0.05)
        {
            lidar_response.data = 1;
            break;
        }
    }

    //Check the right third
    for(int i = right_start; i < right_end; i++)
    {
        //Long range avoidance
        if(msg->ranges[i] < thres_long && msg->ranges[i] > 0.05)
        {
            lidar_response.data = -1;
            break;
        }
    }

    //Short Range Avoidance Left 
    for(int i = (mid-(90/angle_increment)); i < mid; i++)
    {
        if(msg->ranges[i] < thres_short && msg->ranges[i] > 0.05)
        {
            lidar_response.data = 1;
            break;
        }
    }

    //Short Range Avoidance Right
    for(int i = mid; i < (mid+(90/angle_increment)); i++)
    {
        if(msg->ranges[i] < thres_short && msg->ranges[i] > 0.05)
        {
            lidar_response.data = -1;
            break;
        }
    }

    //Check the middle third
    for(int i = left_end; i < right_start; i++)
    {
        lidar_mid_values[i - left_end] = msg->ranges[i];
        if(msg->ranges[i] < thres_long && msg->ranges[i] > 0.05)
        {
            lidar_response.data = 2;
        }
    }
    lidar_front_size = right_start - left_end;

    
    // Nowcheck yomama
    
    // -1 obstacle on right, 1 obstacle on left, 2 obstacle in front, 0 no obstacle wooohoooo
    lidar_pub.publish(lidar_response);
}

bool front_object_service_call(lidar_calcs::front_object_distance::Request &req, lidar_calcs::front_object_distance::Response &res) {
    // Loop through the mid values and check it ........... pray we dont get a lidar callback in the meantime 0_0
    // nah jk callbacks cant happen while we are in here
    

    // Basic logic is that if we find 3 consecutive points with relatively similar values, that will be our object in front of us
    // Threshold is +- 200mm
    float smallest = 150.0;
    for (int i = 0; i < lidar_front_size - 1; i++) {
        float current = lidar_mid_values[i];
        
        // Discard any erroneous values
        if (current > 0.05 && current < 35) {
            if (current < smallest) smallest = current;
        }
    }

    res.object_distance = smallest;
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_response_node");

    ros::NodeHandle nh;

    // Setup subscription to lidar
    ros::Subscriber lidar_get = nh.subscribe("sick_tim_7xx/scan", 10, lidarCallback);
    lidar_pub = nh.advertise<std_msgs::Int16>("lidar_response", 10);

    // Setup service provider
    ros::ServiceServer object_distance_service = nh.advertiseService("lidar_response_node/get_front_object_distance", front_object_service_call);

    ros::spin();

    return 0;
}