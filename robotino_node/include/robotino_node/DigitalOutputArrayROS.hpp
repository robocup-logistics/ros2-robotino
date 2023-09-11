#ifndef DIGITALOUTPUTARRAYROS_HPP_
#define DIGITALOUTPUTARRAYROS_HPP_

#include "rec/robotino/api2/DigitalOutputArray.h"

#include "rclcpp/rclcpp.hpp"
#include "robotino_msgs/msg/digital_readings.hpp"

class DigitalOutputArrayROS: public rec::robotino::api2::DigitalOutputArray
{
public:
	DigitalOutputArrayROS(rclcpp::Node* node);
	~DigitalOutputArrayROS();

private:
	rclcpp::Node* node_;
	rclcpp::Subscription<robotino_msgs::msg::DigitalReadings>::SharedPtr digital_sub_;

	void setDigitalValuesCallback(const robotino_msgs::msg::DigitalReadings::ConstSharedPtr& msg);
};

#endif /* DIGITALOUTPUTARRAYROS_HPP_ */