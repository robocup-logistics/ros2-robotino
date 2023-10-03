#ifndef ANALOGINPUTARRAYROS_HPP_
#define ANALOGINPUTARRAYROS_HPP_

#include "rec/robotino/api2/AnalogInputArray.h"

#include "rclcpp/rclcpp.hpp"
#include "rto_msgs/msg/analog_readings.hpp"

class AnalogInputArrayROS: public rec::robotino::api2::AnalogInputArray
{
public:
	AnalogInputArrayROS(rclcpp::Node* node);
	~AnalogInputArrayROS();

private:
    rclcpp::Node* node_;
    rclcpp::Publisher<rto_msgs::msg::AnalogReadings>::SharedPtr analog_pub_;
	rto_msgs::msg::AnalogReadings analog_msg_;

	void valuesChangedEvent(const float* values, unsigned int size);

};

#endif /* ANALOGINPUTARRAYROS_HPP_ */