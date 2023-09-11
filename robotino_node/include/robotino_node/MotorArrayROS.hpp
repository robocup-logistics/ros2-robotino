#ifndef MOTORARRAYROS_H_
#define MOTORARRAYROS_H_

#include "rec/robotino/api2/MotorArray.h"

#include "rclcpp/rclcpp.hpp"
#include "robotino_msgs/msg/motor_readings.hpp"

class MotorArrayROS : public rec::robotino::api2::MotorArray
{
public:
	MotorArrayROS(rclcpp::Node* node);
	~MotorArrayROS();

	void getMotorReadings(std::vector<float> &velocities, std::vector<int> &positions);

private:
	rclcpp::Node* node_;
	rclcpp::Publisher<robotino_msgs::msg::MotorReadings>::SharedPtr motor_pub_;
	robotino_msgs::msg::MotorReadings motor_msg_;

	void velocitiesChangedEvent(const float* velocities, unsigned int size);
	void positionsChangedEvent(const int* positions, unsigned int size);
	void currentsChangedEvent(const float* currents, unsigned int size);
};

#endif /* MOTORARRAYROS_H_ */