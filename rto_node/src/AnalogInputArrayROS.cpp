#include "rto_node/AnalogInputArrayROS.hpp"

AnalogInputArrayROS::AnalogInputArrayROS(rclcpp::Node* node) : node_(node)
{
    analog_pub_ = node_->create_publisher<rto_msgs::msg::AnalogReadings>("analog_readings", 10);
}

AnalogInputArrayROS::~AnalogInputArrayROS()
{
}

void AnalogInputArrayROS::valuesChangedEvent(const float* values, unsigned int size)
{
	// Build the AnalogReadings msg
	analog_msg_.header.stamp = node_->now();
	analog_msg_.values.resize(size);

	if(size > 0)
	{
		memcpy(analog_msg_.values.data(), values, size * sizeof(float));

		analog_pub_->publish(analog_msg_);
	}
}