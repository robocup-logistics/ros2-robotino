#include "rto_node/EncoderInputROS.hpp"

EncoderInputROS::EncoderInputROS(rclcpp::Node* node) : node_(node)
{
	encoder_pub_ = node_->create_publisher<robotino_msgs::msg::EncoderReadings>("encoder_readings", 10);
	encoder_position_server_ = node_->create_service<robotino_msgs::srv::SetEncoderPosition>("set_encoder_position", std::bind(&EncoderInputROS::setEncoderPositionCallback, this, std::placeholders::_1, std::placeholders::_2));
}

EncoderInputROS::~EncoderInputROS()
{
}

void EncoderInputROS::readingsChangedEvent(int velocity, int position, float current)
{
	// Build the EncoderReadings msg
	encoder_msg_.stamp.stamp = node_->now();
	encoder_msg_.velocity = velocity;
	encoder_msg_.position = position;
	encoder_msg_.current = current;

	// Publish the msg
	encoder_pub_->publish(encoder_msg_);
}

void EncoderInputROS::setEncoderPositionCallback(
	const std::shared_ptr<robotino_msgs::srv::SetEncoderPosition::Request> req,
	[[maybe_unused]] std::shared_ptr<robotino_msgs::srv::SetEncoderPosition::Response> res)
{
	setPosition(req->position ,req->velocity);
}