#include "robotino_node/DigitalOutputArrayROS.hpp"

DigitalOutputArrayROS::DigitalOutputArrayROS(rclcpp::Node* node) : node_(node)
{
	digital_sub_ = node_->create_subscription<robotino_msgs::msg::DigitalReadings>(
      "set_digital_values", 10, std::bind(&DigitalOutputArrayROS::setDigitalValuesCallback, this, std::placeholders::_1));
}

DigitalOutputArrayROS::~DigitalOutputArrayROS()
{
}

void DigitalOutputArrayROS::setDigitalValuesCallback(const robotino_msgs::msg::DigitalReadings::ConstSharedPtr& msg)
{
	if(msg->values.size() == numDigitalOutputs())
	{
		int values[numDigitalOutputs()];
		unsigned int i;

		for(i = 0; i < numDigitalOutputs(); i++)
			values[i] = msg->values[i];

		setValues(values, numDigitalOutputs());
	}
}