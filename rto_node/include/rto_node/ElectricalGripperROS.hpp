#ifndef ELECTRICALGRIPPERROS_HPP_
#define ELECTRICALGRIPPERROS_HPP_

#include "rec/robotino/api2/ElectricalGripper.h"

#include "rclcpp/rclcpp.hpp"
#include "robotino_msgs/msg/gripper_state.hpp"
#include "robotino_msgs/srv/set_gripper_state.hpp"

class ElectricalGripperROS : public rec::robotino::api2::ElectricalGripper
{
public:
	ElectricalGripperROS(rclcpp::Node* node);
	~ElectricalGripperROS();

private:
	rclcpp::Node* node_;
	rclcpp::Publisher<robotino_msgs::msg::GripperState>::SharedPtr gripper_pub_;
	rclcpp::Service<robotino_msgs::srv::SetGripperState>::SharedPtr set_gripper_server_;
	robotino_msgs::msg::GripperState gripper_msg_;

	void setGripperStateCallback(
		const std::shared_ptr<robotino_msgs::srv::SetGripperState::Request> req,
		std::shared_ptr<robotino_msgs::srv::SetGripperState::Response> res);

	void stateChangedEvent(int state);
};

#endif /* ELECTRICALGRIPPERROS_HPP_ */