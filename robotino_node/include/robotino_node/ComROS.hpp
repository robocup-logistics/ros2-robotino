#ifndef COMROS_H_
#define COMROS_H_

#include "rec/robotino/api2/Com.h"

#include "rclcpp/rclcpp.hpp"

class ComROS: public rec::robotino::api2::Com
{
public:
	ComROS();
	~ComROS();

	void setName( const std::string& name );

private:
	std::string name_;

	void errorEvent( const char* errorString );
	void connectedEvent();
	void connectionClosedEvent();
};

#endif /* COMROS_H_ */