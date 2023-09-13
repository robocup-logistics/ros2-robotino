#include "rto_node/ElectricalGripperROS.hpp"

ElectricalGripperROS::ElectricalGripperROS(rclcpp::Node* node) : node_(node)
{
	gripper_pub_ = node_->create_publisher<robotino_msgs::msg::GripperState>("gripper_state", 10);
	set_gripper_server_ = node_->create_service<robotino_msgs::srv::SetGripperState>("set_gripper_state", std::bind(&ElectricalGripperROS::setGripperStateCallback, this, std::placeholders::_1, std::placeholders::_2));
}

ElectricalGripperROS::~ElectricalGripperROS()
{
}

void ElectricalGripperROS::setGripperStateCallback(
	const std::shared_ptr<robotino_msgs::srv::SetGripperState::Request> req,
	[[maybe_unused]] std::shared_ptr<robotino_msgs::srv::SetGripperState::Response> res)
{
	if(req->state)
		open();
	else
		close();
}

void ElectricalGripperROS::stateChangedEvent(int state)
{
	// Build the GripperState msg
	gripper_msg_.stamp.stamp = node_->now();
	if(state == ElectricalGripper::IsOpen)
		gripper_msg_.state = true;
	else
		gripper_msg_.state = false;

	// Publish the msg
	gripper_pub_->publish(gripper_msg_);
}