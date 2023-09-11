#include "robotino_node/DigitalInputArrayROS.hpp"

DigitalInputArrayROS::DigitalInputArrayROS(rclcpp::Node* node) : node_(node)
{
    digital_pub_ = node_->create_publisher<robotino_msgs::msg::DigitalReadings>("digital_readings", 10);
}

DigitalInputArrayROS::~DigitalInputArrayROS()
{
}

void DigitalInputArrayROS::valuesChangedEvent(const int* values, unsigned int size)
{
	// Build the DigitalReadings msg
	digital_msg_.stamp.stamp = node_->now();
	digital_msg_.values.resize(size);

	if(size > 0)
	{
        for(unsigned int idx = 0; idx < size; ++idx)
        {
            digital_msg_.values[idx] = (bool)values[idx];
        }
		// Publish the msg
		digital_pub_->publish(digital_msg_);
	}
}