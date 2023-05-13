#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <global_controller/controller_states.h>
#include "drive_values/enable_remote.h"
#include <std_msgs/Bool.h>

bool driving_enabled  = true;

class RemoteDrive
{
public:
  RemoteDrive();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void updateState(const global_controller::controller_states::ConstPtr& states);
//   bool triggered_driving_enabled_change(drive_values::enable_remote::Request  &req, drive_values::enable_remote::Response &res);

  ros::NodeHandle nh_;

//   bool is_remote_driving   = true;
//   bool is_allowed_to_drive = false;
	bool is_deadman_enabled;

  int linear_, angular_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_, deadman_pub_;
  ros::Subscriber joy_sub_, states_sub_;

  ros::ServiceServer driving_enabled_srv;

  static bool triggered_driving_enabled_change(drive_values::enable_remote::Request  &req, drive_values::enable_remote::Response &res) {
	driving_enabled = req.enabled;
	res.current_state = driving_enabled;
	return true;
}
};


RemoteDrive::RemoteDrive():
  linear_(1),
  angular_(2)

{
    nh_.param("axis_linear", linear_, linear_);
    nh_.param("axis_angular", angular_, angular_);
    nh_.param("scale_angular", a_scale_, a_scale_);
    nh_.param("scale_linear", l_scale_, l_scale_);

    // Setup publishing to RosAria/cmd_vel topic
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1);
    // Setup subscribing to joystick topic
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &RemoteDrive::joyCallback, this);

	deadman_pub_ = nh_.advertise<std_msgs::Bool>("drive_values/deadman_state", 10);

    // Setup service
    driving_enabled_srv = nh_.advertiseService("drive_values/enable_remote", triggered_driving_enabled_change);

    // Setup subscribing to global_controller/states topic
    // states_sub_ = nh_.subscribe<global_controller::controller_states>(
    //     "/global_controller/states", 10, &RemoteDrive::updateState, this);
}

// void RemoteDrive::updateState(const global_controller::controller_states::ConstPtr& states) {
//     is_remote_driving   = !states->is_driving_automatically;
//     is_allowed_to_drive = states->is_allowed_to_drive;
// }

void RemoteDrive::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	geometry_msgs::Twist twist;

	twist.angular.z = 0;
	twist.linear.x = 0;

	int turboFactor = 1;

	// Turbo speed when L2 is pressed
	if (joy->buttons[6] == 1) {
		turboFactor = 6;
 	}

    twist.angular.z = a_scale_*joy->axes[angular_]*turboFactor;
    twist.linear.x = l_scale_*joy->axes[linear_]*turboFactor;


	is_deadman_enabled = (joy->buttons[7] == 1) ? true : false;
	std_msgs::Bool deadmanMsg;
	deadmanMsg.data = is_deadman_enabled; 
	deadman_pub_.publish(deadmanMsg);

    // Only do remote driving if the remote_driving state is true
    if (driving_enabled && is_deadman_enabled) {
        vel_pub_.publish(twist);
    }
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "drive_values");
	RemoteDrive drive_values;

	ros::spin();
}
 
