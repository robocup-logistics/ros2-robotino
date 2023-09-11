#ifndef OMNIDRIVEROS_HPP_
#define OMNIDRIVEROS_HPP_

#include "rec/robotino/api2/OmniDrive.h"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

class OmniDriveROS: public rec::robotino::api2::OmniDrive
{
public:
	OmniDriveROS(rclcpp::Node* node);
	~OmniDriveROS();

	void setMaxMin(double max_linear_vel, double min_linear_vel,
		double max_angular_vel, double min_angular_vel);

private:
	rclcpp::Node* node_;
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

	double max_linear_vel_;
	double min_linear_vel_;
	double max_angular_vel_;
	double min_angular_vel_;

	void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
};

#endif /* OMNIDRIVEROS_HPP_ */