#ifndef OMNIDRIVEROS_HPP_
#define OMNIDRIVEROS_HPP_

#include "rec/robotino/api2/OmniDrive.h"
#include "rec/robotino/api2/OmniDriveModel.h"
#include "rto_node/MotorArrayROS.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rto_msgs/srv/set_omni_drive_enabled.hpp"
#include "rto_msgs/msg/motor_error_readings.hpp"
#include "std_msgs/msg/bool.hpp"

class OmniDriveROS: public rec::robotino::api2::OmniDrive
{
public:
	OmniDriveROS(rclcpp::Node* node);
	~OmniDriveROS();

	void setMaxMin(double max_linear_vel, double min_linear_vel,
		double max_angular_vel, double min_angular_vel);

	void setBumperTime(double period_sec);

private:
	rclcpp::Node* node_;
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
	rclcpp::Service<rto_msgs::srv::SetOmniDriveEnabled>::SharedPtr set_enabled_srv_;
	rclcpp::Publisher<rto_msgs::msg::MotorErrorReadings>::SharedPtr motor_error_pub_;
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr bumper_sub_;

	rto_msgs::msg::MotorErrorReadings motor_error_msg_;

	double max_linear_vel_;
	double min_linear_vel_;
	double max_angular_vel_;
	double min_angular_vel_;
	bool bumperhit_prev_state = false;
	bool bumperhit_current_state = false;
	bool enabled_ = true;
	bool bumper_hit=false;
	rclcpp::TimerBase::SharedPtr timer_;
	double bumper_timeout_ = 2.0;
	std::array<float, 3> mSetVelocities;
	std::vector<float> mGetVelocities;
	std::vector<int> mGetPositions;
	std::vector<rclcpp::Time> motor_error_timestamps_;
	double motor_timout_ = 10.0;
	std::vector<bool> prevMotorErrorState_;
	std::vector<bool> currentMotorErrorState_;

	rec::robotino::api2::OmniDriveModel omniDriveModel_;
	MotorArrayROS motorArray_;

	void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
	void initMsgs();
	bool handleSetOmniDriveEnabled(const std::shared_ptr<rto_msgs::srv::SetOmniDriveEnabled::Request> request,
                          std::shared_ptr<rto_msgs::srv::SetOmniDriveEnabled::Response> response);

	void bumperCallback(const std_msgs::msg::Bool::SharedPtr msg);
	void timerCallback();
};

#endif /* OMNIDRIVEROS_HPP_ */
