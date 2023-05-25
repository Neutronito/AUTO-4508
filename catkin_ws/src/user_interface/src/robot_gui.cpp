#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>


// publishing odometry for nav stack
// navigation stack uses tf to determine the robot's location in the world and relate sensor data to a static map

// odom frame is reference frame

nav_msgs::OccupancyGrid occupancygrid;

void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    occupancygrid.data = msg->ranges;
}
int main(int argc, char** argv) {
    // publishing both a transform from the "odom" coordinate frame to the child (base_link) coordinate frame and a nav_msgs/Odometry message

    ros::init(argc, argv, "odometry_publisher");
    ros::NodeHandle n;

    // create both a ros::Publisher and a tf::TransformBroadcaster to send messages out using ROS and tf
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster odom_broadcaster;

    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("odom", 50);

    ros::Publisher laser_pub = n.advertise<nav_msgs::OccupancyGrid>("odom", 50);
    // tf::TransformBroadcaster odom_broadcaster;


    // robot starts at the origin of the "odom" coordinate frame initially
    double x = 0.0;
    double y = 0.0;
    double th = 0.0;

    // velocities that will cause the child frame to move in the "odom" frame at a rate of 0.1m/s
    double vx = 0.1;
    double vy = 0.1;
    double vth = 0.1;

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    // publish odometry information at a rate of 1Hz
    ros::Rate r(1.0);
    while(n.ok()) {
        // tiz ==
        ros::spinOnce();

        // compute odometry in a typical way given the velocities of the robot
        double dt = (current_time - last_time).toSec();
        double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
        double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
        double delta_th = vth * dt;

        x += delta_x;
        y += delta_y;
        th += delta_th;

        // since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

        // publish the transform over tf
        // transformStamped expresses a transform from coordinate frame header.frame_id to the coordinate frame child_frame_id
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";

        // child frame is coordinate frame we're sending our velocity information in
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        odom_broadcaster.sendTransform(odom_trans);

        // publish the odometry message over ROS
        // nav_msgs/Odometry Message (http://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html)
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        //set the position
        // point position and quaternion position
        // http://docs.ros.org/en/api/geometry_msgs/html/msg/Point.html -> http://docs.ros.org/en/api/geometry_msgs/html/msg/Pose.html -> http://docs.ros.org/en/api/geometry_msgs/html/msg/PoseWithCovariance.html -> http://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat; // quaternion orientation 

        // // set velocit
        // odom.child_frame_id = "base_link"
        // odom.twist.twist.linear.x = vx;
        // odom.twist.twist.linear.y = vy;
        // odom.twist.twist.angular.z = vth;

        // nav_msgs::Path path;
        // path.header.stamp = current_time;
        // path.header.frame_id = "odom";

        // path.poses.pose.pose.position.x = x;
        // path.poses.pose.pose.position.y = y;
        // path.poses.pose.pose.position.z = z;
        // path.poses.pose.pose.position.odom_quat = odom_quat;

        // path.poses->pose


        ////////////////////////////////////////////////////////////////////////
        geometry_msgs::TransformStamped laser_to_odom_trans;
        laser_to_odom_trans.header.stamp = current_time;
        laser_to_odom_trans.header.frame_id = "odom";
        laser_to_odom_trans.child_frame_id = "LaserScan";

        laser_to_odom_trans.translation.x = x;
        laser_to_odom_trans.translation.y = y;
        laser_to_odom_trans.translation.z = 0.0; 
        laser_to_odom_trans.rotation = odom_quat;

        odom_broadcaster.sendTransform(laser_to_odom_trans);


        // nav_msgs::OccupancyGrid occupancygrid;
        occupancygrid.header.stamp = current_time;
        occupancygrid.header.frame_id = "odom";

        occupancygrid.info.map_load_time = current_time;
        occupancygrid.info.resolution = 1;
        occupancygrid.info.width = 500;
        occupancygrid.info.height = 500;
        occupancygrid.info.origin.position.x = 250.0;
        occupancygrid.info.origin.position.y = 250.0;
        occupancygrid.info.origin.position.z = 0.0;

        occupancygrid.info.origin.orientation.x = 0.0;
        occupancygrid.info.origin.orientation.y = 0.0;
        occupancygrid.info.origin.orientation.z = 0.0;
        occupancygrid.info.origin.orientation.w = 0.0;

        ros::Subscriber lidar_sub = n.subscribe("sick_tim_7xx/scan", 10, lidarCallback);


        // publish
        odom_pub.publish(odom);
        // path_pub.publish(path);
        laser_pub.publish(occupancygrid);
        last_time = current_time;
        r.sleep();
    }
}

