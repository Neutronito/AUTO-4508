#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>


class RemoteDrive
{
public:
  RemoteDrive();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  int linear_, angular_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;

};


RemoteDrive::RemoteDrive():
  linear_(1),
  angular_(2)

{

  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);


  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1);


  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &RemoteDrive::joyCallback, this);

}

void RemoteDrive::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	geometry_msgs::Twist twist;

	twist.angular.z = 0;
	twist.linear.x = 0;

	int turboFactor = 1;

	// Turbo speed when L2 is pressed
	if (joy->buttons[6] == 1) {
		turboFactor = 4;
 	}

	// Deadman switch on R2
	if (joy->buttons[7] == 1) {
		twist.angular.z = a_scale_*joy->axes[angular_]*turboFactor;
		twist.linear.x = l_scale_*joy->axes[linear_]*turboFactor;
	} 

	vel_pub_.publish(twist);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "drive_values");
  RemoteDrive drive_values;

  ros::spin();
}
 
