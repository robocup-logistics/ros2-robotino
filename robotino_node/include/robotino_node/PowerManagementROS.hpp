#ifndef POWERMANAGEMENTROS_H_
#define POWERMANAGEMENTROS_H_

#include "rec/robotino/api2/PowerManagement.h"

#include "rclcpp/rclcpp.hpp"
#include "robotino_msgs/msg/power_readings.hpp"

class PowerManagementROS: public rec::robotino::api2::PowerManagement
{
public:
	PowerManagementROS(rclcpp::Node* node);
	virtual ~PowerManagementROS();

private:
	rclcpp::Node* node_;
	void readingsEvent(float current, float voltage);
	void powerTimerCb();

	rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<robotino_msgs::msg::PowerReadings>::SharedPtr publisher_;
};
#endif /* POWERMANAGEMENTROS_H_ */