#include "rto_node/ComROS.hpp"
#include <sstream>

ComROS::ComROS()
{
}

ComROS::~ComROS()
{
}

void ComROS::setName(const std::string& name)
{
	name_ = name;
}

void ComROS::errorEvent(const char* errorString)
{
	std::ostringstream os;
	os << name_ << " : " << errorString;
	RCLCPP_INFO(rclcpp::get_logger("rto_node"), os.str().c_str());
}

void ComROS::connectedEvent()
{
	std::ostringstream os;
	os << name_ << " connected to Robotino.";
	RCLCPP_INFO(rclcpp::get_logger("rto_node"), os.str().c_str());
}

void ComROS::connectionClosedEvent()
{
	std::ostringstream os;
	os << name_ << " disconnected from Robotino.";
	RCLCPP_INFO(rclcpp::get_logger("rto_node"), os.str().c_str());
}
