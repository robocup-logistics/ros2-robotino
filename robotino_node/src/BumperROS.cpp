#include "robotino_node/BumperROS.hpp"

BumperROS::BumperROS(rclcpp::Node* node) : node_(node)
{
    bumper_pub_ = node_->create_publisher<std_msgs::msg::Bool>("bumper", 10);
}

BumperROS::~BumperROS()
{
}

void BumperROS::bumperEvent(bool hasContact)
{
	bumper_msg_.data = hasContact;
	bumper_pub_->publish(bumper_msg_);
}