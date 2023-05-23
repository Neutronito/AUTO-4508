#include "ros/ros.h"
#include "stereo_camera_testing/object_locations.h"
#include "pioneer_driver/action_requests.h"
#include "pioneer_driver/object_detection_finished.h"
#include <time.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include "lidar_calcs/front_object_distance.h"

// Driving stuff
float linearVelocity = 0;
float angularVelocity = 0;

// Task state variables
bool onAJob = false;
bool currentlyPaused = false;
bool deadman_pressed = false;

// State 0 indicates rotating on the stop to find a cone or bucket
// State 1 indicates positioning and obtaining the distance of the cone
// State 2 indicates positioning and obtaining the distance of a bucket
int state = 0;
int pseudoDelay = 0;

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


// This is for task 0, indicates whether we should start rotating or take a photo and a timer
bool onSpotRotating = false;
bool takeAnImage = false;
double rotatingStartTime;
double timeToSpendRotating = 0;

// Positioning stuff
#define IMAGE_CENTRE_X 416
#define CENTRE_TOLERANCE 20
// Distance threshold is in mm
#define DISTANCE_THRESHOLD 1000


// Received a command from an external node
void received_command(const pioneer_driver::action_requests::ConstPtr& msg) {
    // Start a job
    if (msg->start_cone_detection) {
        onAJob = true;
        state = 0;
        ROS_INFO("Starting a cone job");
    }
    // Deal with pausing
    currentlyPaused = msg->pause_cone_detection;
    ROS_INFO("Cone paused state is %s", currentlyPaused ? "true" : "false");
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

// Check if the deadman is pressed
void deadman_callback(const std_msgs::Bool::ConstPtr& msg) {
	deadman_pressed = msg->data;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "cone_driving_detection");
    ros::NodeHandle cone_handle;

    // Start the cone detecion topic, this listens for start requests, end requests and pause requests
    ros::Subscriber command_listener = cone_handle.subscribe("cone_driving_detection/action_requests", 10, received_command);

    // Setup publishing to rosaria cmd_vel
	ros::Publisher velocity_publisher = cone_handle.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 10);

    // Setup publishing to finished state
    ros::Publisher finished_publisher = cone_handle.advertise<pioneer_driver::object_detection_finished>("cone_driving_detection/finished_state", 10);
    
    // Setup client for camera recognition
    ros::ServiceClient camera_client = cone_handle.serviceClient<stereo_camera_testing::object_locations>("stereo_camera_testing/object_locations");

    // Subscribe to imu to get our current heading
	ros::Subscriber imu_subscriber = cone_handle.subscribe("imu_heading", 10, imu_heading_callback);

    // Setup client for distance finding
    ros::ServiceClient distance_client = cone_handle.serviceClient<lidar_calcs::front_object_distance>("lidar_response_node/get_front_object_distance");
    
    // Setup subscription to deadman topic
	ros::Subscriber deadman_subscriber = cone_handle.subscribe("drive_values/deadman_state", 10, deadman_callback);
    
    // Run at 10Hz
    ros::Rate loopRate(10);
    
    ROS_INFO("Cone driving detection node successfully initialized");

    while (ros::ok()){
        
        // Check if I'm on a job and not paused
        if (onAJob && !currentlyPaused && deadman_pressed) {

            // Rotating finding cone or bucket state~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            if (state == 0) {

                // Check if we have finished
                if (coneFound && bucketFound) {
                    // Find the distance between the objects
                    // Note angles are 0 to 360 (yay easier maths)
                    float angleBetweenObjects = abs(coneAngle - bucketAngle);

                    // use cosine law
                    float distance = sqrt((coneDistance * coneDistance) + (bucketDistance * bucketDistance) - 2 * coneDistance * bucketDistance * cos(angleBetweenObjects));

                    // Now publish the information
                    pioneer_driver::object_detection_finished msg;
                    msg.finished_job_successfully = true;
                    msg.bucket_cone_distance = distance;

                    finished_publisher.publish(msg);

                    ROS_INFO("The cone node has terminated, identifying a cone and a bucket with a distance apart of %fmm", distance);
                    onAJob = false;
                }

                
                // Check if I need to take a photo
                if (takeAnImage) {
                    // Send a service request
                    ROS_INFO("Cone driver is requesting an image from the stereo node");
                    stereo_camera_testing::object_locations camSrv;
                    camSrv.request.dummy_var = true;
                    
                    // Check if we have found an object
                    if (camera_client.call(camSrv)) {

                        // Check if we have found a cone
                            if (camSrv.response.found_cone) {
                                // check if we have found the cone already or not
                                if (!coneFound) {
                                    ROS_INFO("A cone has been found, moving to state one");
                                    state = 1;
                                }
                            } 
                            
                            // Check if we have foudn the bucket
                            else if (camSrv.response.found_bucket) {
                                // Only move to bucket state if the cone has been found
                                if (coneFound) {
                                    ROS_INFO("A bucket has been found, moving to state two");
                                    state = 2;
                                }
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
                    ROS_INFO("Now rotating in search of an object");
                    
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
                    if (camera_client.call(camSrv)) {
                        int coneX = camSrv.response.cone_x;

                        // Find out how far the cone is from centre
                        int centre_offset = IMAGE_CENTRE_X - coneX;
                        linearVelocity = 0;

                        //Check if the cone is close enough to the centre or not
                        if (abs(centre_offset) < CENTRE_TOLERANCE) {
                            // Now check if the cone is close enough or not
                            if (camSrv.response.cone_z >  DISTANCE_THRESHOLD) {
                                linearVelocity = 1;
                                pseudoDelay = 0;
                            } else if (pseudoDelay < 3) {
                                pseudoDelay++;
                            }
                            // Now take a photo of the cone and leave this state
                            else {
                                // Take photo of cone
                                coneDistance = camSrv.response.cone_z;
                                coneFound = true;
                                coneAngle = currentOrientation;
                                state = 0;
                            }
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

                    ROS_INFO("Rotating to centre cone");
                    
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
                    if (camera_client.call(camSrv)) {
                        int bucketX = camSrv.response.bucket_x;

                        // Find out how far the cone is from centre
                        int centre_offset = IMAGE_CENTRE_X - bucketX;

                        //Check if the cone is close enough to the centre or not
                        if (abs(centre_offset) < CENTRE_TOLERANCE) {
                            bucketDistance = camSrv.response.bucket_z;
                            bucketAngle = currentOrientation;
                            bucketFound = true;
                            state = 0;
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

                    ROS_INFO("Rotating to centre bucket");
                    
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