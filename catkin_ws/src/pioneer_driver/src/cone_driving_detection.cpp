#include "ros/ros.h"
#include "stereo_camera_testing/object_locations.h"
#include "pioneer_driver/action_requests.h"
#include <time.h>
#include <geometry_msgs/Twist.h>

// Driving stuff
float linearVelocity = 0;
float angularVelocity = 0;

// Task state variables
bool onAJob = false;
bool currentlyPaused = false;

// State 0 indicates rotating on the stop to find a cone
int state = 0;

// This is for task 0, indicates whether we should start rotating or take a photo and a timer
bool onSpotRotating = false;
bool takeAnImage = false;
double rotatingStartTime;
#define ROTATION_DURATION


// Received a command from an external node
void received_command(const pioneer_driver::action_requests::ConstPtr& msg) {
    ROS_INFO("Recieved a request to start cone detection");
    // Start a job
    if (msg->start_cone_detection) {
        onAJob = true;
        state = 0;
        ROS_INFO("Starting a cone job");
    }
    // Deal with pausing
    currentlyPaused = msg->pause_cone_detection;
    // Deal with terminating a job
    if (msg->terminate_current_detection) {
        onAJob = false;
        ROS_INFO("Terminating cone job");
    }
} 

int main(int argc, char **argv) {
    ros::init(argc, argv, "cone_driving_detection");
    ros::NodeHandle cone_handle;

    // Start the cone detecion topic, this listens for start requests, end requests and pause requests
    ros::Subscriber command_listener = cone_handle.subscribe("cone_driving_detection/action_requests", 10, received_command);

    // Setup publishing to rosaria cmd_vel
	ros::Publisher velocity_publisher = cone_handle.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 10);

    // Setup client for camera recognition
    ros::ServiceClient client = cone_handle.serviceClient<stereo_camera_testing::object_locations>("stereo_camera_testing/object_locations");

    // Run at 10Hz
    ros::Rate loopRate(10);
    
    // I ADDED THIS TO MAKE IT RUN
    // COMMAND LISTENER SUBSCRIBER SEEMS TO NOT GET TOPIC MESSAGES

    while (ros::ok()){

        
        
        // Check if I'm on a job and not paused
        if (onAJob && !currentlyPaused) {
            // ROS_INFO("WGAOSJD");

            // I'm currently rotating
            if (state == 0) {

                
                // Check if I need to take a photo
                if (takeAnImage) {
                    // Send a service request
                    ROS_INFO("Cone driver is requesting an image from the stereo node");
                    stereo_camera_testing::object_locations camSrv;
                    camSrv.request.dummy_var = true;

                    if (client.call(camSrv)) {
                        ROS_INFO("Success calling object_locations service");

                        // cone has been detected
                        if (camSrv.response.found_cone) {
                            ROS_INFO("A cone has been found, exiting state one");
                            onAJob = false;
                        } else {
                            ROS_INFO("Failed to find a cone, I will keep rotating");
                        }
                    } else {
                        ROS_ERROR("Failed to call service object_locations");
                    }
                    takeAnImage = false;                    
                } 
                // Check if I'm not rotating on the spot, and if not start doing it
                else if(!onSpotRotating) {
                    onSpotRotating = true;
                    linearVelocity = 0;
                    angularVelocity = 1;
                    rotatingStartTime = clock();
                }
                // I'm rotating on the spot, stop after 500 milliseconds
                else {
                    double elapsedTimeMillis = (clock() - rotatingStartTime) / (CLOCKS_PER_SEC / 1000);
                    
                    // TODO: Fix this value, it is not scaled to milliseconds properly
                    if (elapsedTimeMillis > 1) {
                        angularVelocity = 0;
                        onSpotRotating = false;
                        takeAnImage = true;
                    }
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
}