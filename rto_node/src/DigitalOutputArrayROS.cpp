#include "rto_node/DigitalOutputArrayROS.hpp"

DigitalOutputArrayROS::DigitalOutputArrayROS(rclcpp::Node* node) : node_(node)
{
	digital_sub_ = node_->create_subscription<rto_msgs::msg::DigitalReadings>(
      "set_digital_values", 10, std::bind(&DigitalOutputArrayROS::setDigitalValuesCallback, this, std::placeholders::_1));
}

DigitalOutputArrayROS::~DigitalOutputArrayROS()
{
}

void DigitalOutputArrayROS::setDigitalValuesCallback(const rto_msgs::msg::DigitalReadings::ConstSharedPtr& msg)
{
	if(msg->values.size() == numDigitalOutputs())
	{
		std::vector<int> values = std::vector<int>(numDigitalOutputs());

		for(unsigned int i = 0; i < numDigitalOutputs(); i++)
			values.at(i) = msg->values[i];

		setValues(values.data(), numDigitalOutputs());
	}
}