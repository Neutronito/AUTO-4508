#include "ros/ros.h"
#include "stereo_camera_testing/object_locations.h"
#include "pioneer_driver/action_requests.h"
#include <time.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>

// Driving stuff
float linearVelocity = 0;
float angularVelocity = 0;

// Task state variables
bool onAJob = false;
bool currentlyPaused = false;

// State 0 indicates rotating on the stop to find a cone or bucket
// State 1 indicates positioning and obtaining the distance of the cone
// State 2 indicates positioning and obtaining the distance of a bucket
int state = 0;

// Recording variables for cone
int coneDistance = 0;
bool coneFound = false;
float coneAngle = 0;

// Recording variables for bucket
int bucketDistance = 0;
bool bucketFound = 0;
float bucketAngle = 0;

// Note, this orientation is 0 to 360 clockwise with 0 North
int currentOrientation = 0;

// Formula for quaternion
//https://robotics.stackexchange.com/questions/16471/get-yaw-from-quaternion

// This is for task 0, indicates whether we should start rotating or take a photo and a timer
bool onSpotRotating = false;
bool takeAnImage = false;
double rotatingStartTime;
double timeToSpendRotating = 0;

// Positioning stuff
#define IMAGE_CENTRE_X 400
#define CENTRE_TOLERANCE 20


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

// Gets the imu heading
void imu_heading_callback(const std_msgs::Float64::ConstPtr& msg) {
    if (onAJob) {
		currentOrientation = (int)msg->data;
        // Map to 0 - 360
        currentOrientation = (currentOrientation + 360) % 360;
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

    // Subscribe to imu to get our current heading
	ros::Subscriber imu_subscriber = cone_handle.subscribe("imu_heading", 10, imu_heading_callback);
    
    // Run at 10Hz
    ros::Rate loopRate(10);
    
    ROS_INFO("Cone driving detection node successfully initialized");

    while (ros::ok()){
        
        // Check if I'm on a job and not paused
        if (onAJob && !currentlyPaused) {

            // Rotating finding cone state~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            if (state == 0) {

                
                // Check if I need to take a photo
                if (takeAnImage) {
                    // Send a service request
                    ROS_INFO("Cone driver is requesting an image from the stereo node");
                    stereo_camera_testing::object_locations camSrv;
                    camSrv.request.dummy_var = true;
                    
                    // Check if we have found an object
                    if (client.call(camSrv)) {

                        // Check if we have found a cone
                            if (camSrv.response.found_cone) {
                                ROS_INFO("A cone has been found, moving to state one");
                                state = 1;
                            } else if (camSrv.response.found_bucket) {
                                ROS_INFO("A bucket has been found, moving to state two");
                                state = 2;
                            } else {
                                ROS_INFO("No object found, continuing rotation");
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

                    
                    struct timespec currentTime;
                    clock_gettime(CLOCK_MONOTONIC_RAW, &currentTime);
                    rotatingStartTime = currentTime.tv_sec * 1000 ;
                }
                // I'm rotating on the spot, stop after 500 milliseconds
                else {

                    struct timespec currentTime;
                    clock_gettime(CLOCK_MONOTONIC_RAW, &currentTime);
                    double elapsedTimeMillis = (currentTime.tv_sec * 1000 - rotatingStartTime);
                    
                    // Stop rotating
                    if (elapsedTimeMillis > 500) {
                        angularVelocity = 0;
                    }

                    // Give the robot some time to stop rotating
                    if (elapsedTimeMillis > 1000) {
                    onSpotRotating = false;
                        takeAnImage = true;
                    }
                }

            } 
            
            // We are trying to find the position of the cone~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            else if (state == 1) {
                
                // We are not rotating, so that means we should take an image now
                if (!onSpotRotating) {
                
                    stereo_camera_testing::object_locations camSrv;
                    camSrv.request.dummy_var = true;
                    
                    // Call the service to obtain the position of the cone
                    if (client.call(camSrv)) {
                        int coneX = camSrv.response.cone_x;

                        // Find out how far the cone is from centre
                        int centre_offset = IMAGE_CENTRE_X - coneX;

                        //Check if the cone is close enough to the centre or not
                        if (abs(centre_offset) < CENTRE_TOLERANCE) {
                            // Put in dummy code to find the lidar
                        }

                        // If not, figure out how long we should rotate for
                        // negative offset means cone is to the right, which is a negativ rotation (clockwise) so sign works out nicely
                        timeToSpendRotating = abs(centre_offset) * 100;
                        if (centre_offset < 0) {
                            angularVelocity = -1;
                        } else {
                            angularVelocity = 1;
                        }
                        onSpotRotating = true;
                        
                        // Set the start time
                        struct timespec currentTime;
                        clock_gettime(CLOCK_MONOTONIC_RAW, &currentTime);
                        rotatingStartTime = currentTime.tv_sec * 1000 ;                

                    } else {
                        ROS_ERROR("Failed to call service object_locations");
                    }
                } 
                
                // We are currently rotating on the spot, stop after the set time has passed
                else {
                    struct timespec currentTime;
                    clock_gettime(CLOCK_MONOTONIC_RAW, &currentTime);
                    double elapsedTimeMillis = (currentTime.tv_sec * 1000 - rotatingStartTime);
                    
                    // Stop rotating
                    if (elapsedTimeMillis > timeToSpendRotating) {
                        angularVelocity = 0;
                    }

                    // Give the robot some time to stop rotating
                    if (elapsedTimeMillis > (timeToSpendRotating + 500)) {
                        onSpotRotating = false;
                    }
                }

            } 
            
            // We are trying to find the position of the bucket~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            else if (state == 2) {
                // We are not rotating, so that means we should take an image now
                if (!onSpotRotating) {
                
                    stereo_camera_testing::object_locations camSrv;
                    camSrv.request.dummy_var = true;
                    
                    // Call the service to obtain the position of the cone
                    if (client.call(camSrv)) {
                        int bucketX = camSrv.response.bucket_x;

                        // Find out how far the cone is from centre
                        int centre_offset = IMAGE_CENTRE_X - bucketX;

                        //Check if the cone is close enough to the centre or not
                        if (abs(centre_offset) < CENTRE_TOLERANCE) {
                            // Put in dummy code to find the lidar
                        }

                        // If not, figure out how long we should rotate for
                        // negative offset means cone is to the right, which is a negativ rotation (clockwise) so sign works out nicely
                        timeToSpendRotating = abs(centre_offset) * 100;
                        if (centre_offset < 0) {
                            angularVelocity = -1;
                        } else {
                            angularVelocity = 1;
                        }
                        onSpotRotating = true;
                        
                        // Set the start time
                        struct timespec currentTime;
                        clock_gettime(CLOCK_MONOTONIC_RAW, &currentTime);
                        rotatingStartTime = currentTime.tv_sec * 1000 ;                

                    } else {
                        ROS_ERROR("Failed to call service object_locations");
                    }
                } 
                
                // We are currently rotating on the spot, stop after the set time has passed
                else {
                    struct timespec currentTime;
                    clock_gettime(CLOCK_MONOTONIC_RAW, &currentTime);
                    double elapsedTimeMillis = (currentTime.tv_sec * 1000 - rotatingStartTime);
                    
                    // Stop rotating
                    if (elapsedTimeMillis > timeToSpendRotating) {
                        angularVelocity = 0;
                    }

                    // Give the robot some time to stop rotating
                    if (elapsedTimeMillis > (timeToSpendRotating + 500)) {
                        onSpotRotating = false;
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