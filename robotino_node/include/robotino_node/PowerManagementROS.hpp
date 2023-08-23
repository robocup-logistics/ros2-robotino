#ifndef POWERMANAGEMENTROS_H_
#define POWERMANAGEMENTROS_H_

#include "rec/robotino/api2/PowerManagement.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class PowerManagementROS: public rec::robotino::api2::PowerManagement
{
public:
	PowerManagementROS(rclcpp::Node* node);
	virtual ~PowerManagementROS();

private:
	rclcpp::Node* node_;
	void readingsEvent(float current, float voltage);
};
#endif /* POWERMANAGEMENTROS_H_ */