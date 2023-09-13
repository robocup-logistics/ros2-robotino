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
	std::cout << errorString << std::endl;
	//RCLCPP_ERROR("%s", os.str().c_str() );
}

void ComROS::connectedEvent()
{
	std::ostringstream os;
	os << name_ << " connected to Robotino.";
	std::cout << name_ << " connected to Robotino." << std::endl;
	// RCLCPP_INFO("%s", os.str().c_str() );
}

void ComROS::connectionClosedEvent()
{
	std::ostringstream os;
	os << name_ << " disconnected from Robotino.";
	std::cout << name_ << " disconnected from Robotino." << std::endl;
	// RCLCPP_INFO("%s", os.str().c_str() );
}