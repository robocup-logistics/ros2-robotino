#ifndef OMNIDRIVEROS_HPP_
#define OMNIDRIVEROS_HPP_

#include "rec/robotino/api2/OmniDrive.h"
#include "rec/robotino/api2/OmniDriveModel.h"
#include "rec/robotino/api2/MotorArray.h"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rto_msgs/srv/set_omni_drive_enabled.hpp"
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

	bool enabled_ = true;
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr bumper_sub_;

	double max_linear_vel_;
	double min_linear_vel_;
	double max_angular_vel_;
	double min_angular_vel_;
	bool bumperhit_prev_state = false;
	bool bumperhit_current_state = false;
	bool bumper_hit=false;
	rclcpp::TimerBase::SharedPtr timer_;
	double timer_period_ = 2.0;
	std::array<float, 3> mSetVelocities;
	std::array<float, 3> mGetVelocities;

	void project(float *m1, float *m2, float *m3, float vx, float vy, float omega) const
	void velocitiesChangedEvent(const float* velocities, unsigned int size);
	void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

	bool handleSetOmniDriveEnabled(const std::shared_ptr<rto_msgs::srv::SetOmniDriveEnabled::Request> request,
                          std::shared_ptr<rto_msgs::srv::SetOmniDriveEnabled::Response> response);

	void bumperCallback(const std_msgs::msg::Bool::SharedPtr msg);
	void timerCallback();
};

#endif /* OMNIDRIVEROS_HPP_ */
