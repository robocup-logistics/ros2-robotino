#ifndef OMNIDRIVEROS_HPP_
#define OMNIDRIVEROS_HPP_

#include "rec/robotino/api2/OmniDrive.h"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rto_msgs/srv/set_omni_drive_enabled.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
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
	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr ir_scan_sub_;
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr bumper_sub_;

	bool enabled_ = true;

	double max_linear_vel_;
	double min_linear_vel_;
	double max_angular_vel_;
	double min_angular_vel_;
	bool bumper_hit=false;
	rclcpp::TimerBase::SharedPtr timer_;
	double timer_period_ = 2.0;

	rclcpp::TimerBase::SharedPtr velocity_timer_;
	int frequency_ = 20;
    double velocity_duration_ = 4.0;  
    rclcpp::Time velocity_end_time_;
    float x_, y_, theta_;
	bool sensor_check_ = false;

    void velocityTimerCallback();

	void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

	bool handleSetOmniDriveEnabled(const std::shared_ptr<rto_msgs::srv::SetOmniDriveEnabled::Request> request,
                          std::shared_ptr<rto_msgs::srv::SetOmniDriveEnabled::Response> response);
	void bumperCallback(const std_msgs::msg::Bool::SharedPtr msg);
	void timerCallback();

	void irScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

};

#endif /* OMNIDRIVEROS_HPP_ */
