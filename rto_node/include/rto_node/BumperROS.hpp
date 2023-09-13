#ifndef BUMPERROS_HPP_
#define BUMPERROS_HPP_

#include "rec/robotino/api2/Bumper.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

class BumperROS: public rec::robotino::api2::Bumper
{
public:
	BumperROS(rclcpp::Node* node);
	virtual ~BumperROS();

private:
    rclcpp::Node* node_;
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr bumper_pub_;
	std_msgs::msg::Bool bumper_msg_;

	void bumperEvent(bool hasContact);
};

#endif /* BUMPERROS_HPP_ */