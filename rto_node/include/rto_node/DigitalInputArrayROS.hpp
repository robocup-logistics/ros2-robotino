#ifndef DIGITALINPUTARRAYROS_HPP_
#define DIGITALINPUTARRAYROS_HPP_

#include "rec/robotino/api2/DigitalInputArray.h"

#include "rclcpp/rclcpp.hpp"
#include "rto_msgs/msg/digital_readings.hpp"

class DigitalInputArrayROS: public rec::robotino::api2::DigitalInputArray
{
public:
	DigitalInputArrayROS(rclcpp::Node* node);
	~DigitalInputArrayROS();

private:
	rclcpp::Node* node_;
	rclcpp::Publisher<rto_msgs::msg::DigitalReadings>::SharedPtr digital_pub_;
	rto_msgs::msg::DigitalReadings digital_msg_;

	void valuesChangedEvent( const int* values, unsigned int size );
};

#endif /* DIGITALINPUTARRAYROS_HPP_ */