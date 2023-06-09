#include "ros/ros.h"
#include "stereo_camera_testing/object_locations.h"
#include "stereo_camera_testing/take_photo.h"
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

bool banForwards = false;

// State 0 indicates rotating on the stop to find a cone or bucket
// State 1 indicates positioning and obtaining the distance of the cone
// State 2 indicates positioning and obtaining the distance of a bucket
int state = 0;
int pseudoDelay = 0;
int start_time;

// Recording variables for cone
float coneDistance = 0;
bool coneFound = false;
float coneAngle = 0;

// Recording variables for bucket
float bucketDistance = 0;
bool bucketFound = 0;
float bucketAngle = 0;

// Note, this orientation is 0 to 360 clockwise with 0 North
int currentOrientation = 0;
int prevOrientation = 0;
int desiredOrientation = 0;
int rotatedAmount = 0;


// This is for task 0, indicates whether we should start rotating or take a photo and a timer
bool onSpotRotating = false;
bool takeAnImage = false;
double rotatingStartTime;
double timeToSpendRotating = 0;

// Positioning stuff
#define CENTRE_TOLERANCE 20
// Distance threshold is in M
#define DISTANCE_THRESHOLD 2000.0
#define CENTRE_DRIVING_THRESHOLD 100


// Received a command from an external node
void received_command(const pioneer_driver::action_requests::ConstPtr& msg) {
    // Start a job
    if (msg->start_cone_detection) {
        onAJob = true;
        state = 0;
        coneFound = false;
        bucketFound = false;
        banForwards = false;
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
    rotatedAmount = 0;
}

// Gets the imu heading
void imu_heading_callback(const std_msgs::Float64::ConstPtr& msg) {
    if (onAJob) {
		prevOrientation = currentOrientation;
        currentOrientation = (int)msg->data;
        // Map to 0 - 360
        currentOrientation = (currentOrientation + 360) % 360;

        // Keep track of how far we have rotated
        float cw = abs(prevOrientation - currentOrientation);
        float ccw = 360 - cw;

        if (ccw < cw) {
            rotatedAmount += ccw;
        } else {
            rotatedAmount += cw;
        }
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

    // Setup client for distance finding - deprecated
    ros::ServiceClient distance_client = cone_handle.serviceClient<lidar_calcs::front_object_distance>("lidar_response_node/get_front_object_distance");

    // Setup client for requesting an image to be taken
    ros::ServiceClient photo_taker = cone_handle.serviceClient<stereo_camera_testing::take_photo>("stereo_camera_testing/image_request");
    
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

                linearVelocity = 0;

                // Check if we have not found anything
                if (rotatedAmount > 720) {
                    ROS_INFO("The cone node has rotated 720 degrees without finding anything, continuing to next waypoint");
                    onAJob = false;
                    pioneer_driver::object_detection_finished msg;
                    msg.finished_job_successfully = false;
                    msg.bucket_cone_distance = 0;
                    finished_publisher.publish(msg);
                }

                // Send a service request
                ROS_INFO("Cone driver is requesting an image from the stereo node");
                stereo_camera_testing::object_locations camSrv;
                camSrv.request.dummy_var = true;
                angularVelocity = 0.3;

                
                // Check if we have found an object
                if (camera_client.call(camSrv)) {

                    // Check if we have found a cone
                    if (camSrv.response.found_cone && camSrv.response.cone_z > 0) {
                        // check if we have found the cone already or not
                        if (!coneFound) {
                            ROS_INFO("A cone has been found, moving to state one");
                            state = 1;
                            rotatedAmount = 0;
                        } else if (coneFound && bucketFound) {
                            state = 3;
                            rotatedAmount = 0;
                            angularVelocity = 0.5;
                            ROS_INFO("Re-found the cone, now rotate 90* to the left.");
                        }
                    } 
                    
                    // Check if we have foudn the bucket
                    else if (camSrv.response.found_bucket && camSrv.response.bucket_z > 0) {
                        // Only move to bucket state if the cone has been found, and a bucket hasnt been found
                        if (coneFound && !bucketFound) {
                            ROS_INFO("A bucket has been found, moving to state two");
                            state = 2;
                            pseudoDelay = 0;
                            rotatedAmount = 0;
                        }
                    } else {
                        ROS_INFO("No object found, continuing rotation");
                    }
                } else {
                    ROS_ERROR("Failed to call service object_locations");
                }

                lidar_calcs::front_object_distance srv;
                srv.request.dummy_var = true;
                //Now check if an object is too close
                if (distance_client.call(srv)) {
                    if (srv.response.object_distance < 1.0) {
                        ROS_INFO("Cone node is too close to an object, moving backwards for 0.5 seconds");
                        angularVelocity = 0;
                        linearVelocity = -0.5;
                        state = 4;

                        if (coneFound) banForwards = true;
                        
                    }
                } else {
                    ROS_ERROR("Failed to call lidar distance service");
                }

                takeAnImage = false; 
            } 
            
            // We are trying to find the position of the cone~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            else if (state == 1) {

                    stereo_camera_testing::object_locations camSrv;
                    camSrv.request.dummy_var = true;
                    
                    // Call the service to obtain the position of the cone
                    if (camera_client.call(camSrv)) {
                        int centre_offset = camSrv.response.cone_x;

                        if (!camSrv.response.found_cone) {
                            ROS_INFO("Somehow in state 1 but no cone, back to state 0");
                            state = 0;
                            rotatedAmount = 0;
                        }
                        else {

                            // If we are sort of close to the centre, drive to the cone
                            if (abs(centre_offset) < CENTRE_DRIVING_THRESHOLD) {

                                coneDistance = camSrv.response.cone_z;

                                if (coneDistance >  DISTANCE_THRESHOLD && !banForwards) {
                                    linearVelocity = 0.3;
                                    ROS_INFO("Driving at cone, its current distanc is %f", coneDistance);
                                    pseudoDelay = 0;
                                } else {
                                    linearVelocity = 0;
                                    ROS_INFO("Cone detection node is close enough to the cone, and is stopping");
                                }

                                //Check if the cone is close enough to the centre or not
                                if (abs(centre_offset) < CENTRE_TOLERANCE) {
                                    angularVelocity = 0;
                                    // Now check if the cone is close enough or not
                                    if (coneDistance <  DISTANCE_THRESHOLD || banForwards) {
                                        
                                        // Time to take the photo, call the photo taking service
                                        stereo_camera_testing::take_photo srv;
                                        srv.request.dummy_var = true;

                                        if (photo_taker.call(srv)) {
                                            
                                            // This means we successfully took a photo
                                            if (srv.response.photo_status) {
                                                // Update info internally
                                                linearVelocity = 0;
                                                coneDistance = coneDistance;
                                                coneAngle = currentOrientation;
                                                coneFound = true;
                                                ROS_INFO("Found the cone and taken its photo, recording distance and moving on to bucket");
                                                ROS_INFO("The cone distance was found to be %f", coneDistance);

                                                state = 0;
                                                rotatedAmount = 0;
                                            }
                                            
                                            // Photo taking failed, do nothing which means that we will try again next loop
                                            else { }
                                        }
                                        else {
                                            ROS_ERROR("Failed to call service stereo_camera_node/take_image");
                                        }
                                    }
                                }

                                // Now rotate until the cone is centred
                                else {
                                    angularVelocity = (float)abs(centre_offset) / 1800.0;
                                    if (angularVelocity > 1) angularVelocity = 1;
                                    if (angularVelocity < 0.01) angularVelocity = 0.01;
                                    if (centre_offset > 0) angularVelocity *= -1;
                                }
                            }
                            }
                    } else {
                        ROS_ERROR("Failed to call service object_locations");
                    }
            } 
            
            // We are trying to find the position of the bucket~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            else if (state == 2) {
                // We are not rotating, so that means we should take an image now
                stereo_camera_testing::object_locations camSrv;
                camSrv.request.dummy_var = true;

                linearVelocity = 0;

                // Call the service to obtain the position of the cone
                if (camera_client.call(camSrv)) {
                    int centre_offset = camSrv.response.bucket_x;

                    if (!camSrv.response.found_bucket) {
                        state = 0;
                        rotatedAmount = 0;
                        ROS_INFO("Somehow in state 2 but no bucket, moving back to state 0");
                    } else {

                        //Check if the cone is close enough to the centre or not
                        if (abs(centre_offset) < CENTRE_TOLERANCE) {

                            // Time to take the photo, call the photo taking service
                            stereo_camera_testing::take_photo srv;
                            srv.request.dummy_var = true;

                            if (photo_taker.call(srv)) {
                                
                                // This means we successfully took a photo
                                if (srv.response.photo_status) {
                                    // Update internal variables
                                    bucketDistance = camSrv.response.bucket_z;
                                    bucketAngle = currentOrientation;
                                    bucketFound = true;
                                    ROS_INFO("Found the bucket and taken its photo, recording distance and returning to state 0");
                                    state = 0;
                                    rotatedAmount;
                                }
                                // Photo taking failed, do nothing which means that we will try again next loop
                                else { }
                            }
                            else {
                                ROS_ERROR("Failed to call service stereo_camera_node/take_image");
                            }
                        } 
                        
                        // Rotate until the bucket is centred
                        else{
                            angularVelocity = (float)abs(centre_offset) / 1800.0;
                            if (angularVelocity > 1) angularVelocity = 1;
                            if (angularVelocity < 0.01) angularVelocity = 0.01;
                            if (centre_offset > 0) angularVelocity *= -1;
                        }
                    }
                } else {
                    ROS_ERROR("Failed to call service object_locations");
                }
            }

            else if (state == 3) {
                // Check if we've rotated the 90*
                linearVelocity = 0;
                if (rotatedAmount > 90) {
                    angularVelocity = 0;

                    // Find the distance between the objects
                    // Note angles are 0 to 360 (yay easier maths), but just need to convert to rad
                    float angleBetweenObjects = abs(coneAngle - bucketAngle);
                    angleBetweenObjects = angleBetweenObjects / 180 * M_PI;
                    ROS_INFO("The cone angle is %f, the bucket angle is %f and the total angle is %f.", coneAngle, bucketAngle, angleBetweenObjects);
                    
                    
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
            }

            // Moving backwards away from an object that is too close to us
            else if (state == 4) {
                angularVelocity = 0;
                lidar_calcs::front_object_distance srv;
                srv.request.dummy_var = true;
                //Now check if an object is too close
                if (distance_client.call(srv)) {
                    if (srv.response.object_distance > 1) {
                        linearVelocity = 0;
                        state = 0;
                        rotatedAmount = 0;
                    
                        // If we've found a cone or bucket, throw it away, data is no longer valid as we have moved.
                        coneFound = false;
                        bucketFound = false;
                        rotatedAmount = 0;
                    }                
                } else {
                    ROS_ERROR("Failed to call lidar distance service");
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