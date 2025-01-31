#ifndef MOTORARRAYROS_H_
#define MOTORARRAYROS_H_

#include "rec/robotino/api2/MotorArray.h"

#include "rclcpp/rclcpp.hpp"
#include "rto_msgs/msg/motor_readings.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class MotorArrayROS : public rec::robotino::api2::MotorArray
{
public:
	MotorArrayROS(rclcpp::Node* node);
	~MotorArrayROS();

	void getMotorReadings(std::vector<float> &velocities, std::vector<int> &positions);

private:
	rclcpp::Node* node_;
	rclcpp::Publisher<rto_msgs::msg::MotorReadings>::SharedPtr motor_pub_;
	rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;

	rto_msgs::msg::MotorReadings motor_msg_;
	sensor_msgs::msg::JointState joint_state_msg_;
	std::vector<double> preveous_positions_;

	void velocitiesChangedEvent(const float* velocities, unsigned int size);
	void positionsChangedEvent(const int* positions, unsigned int size);
	void currentsChangedEvent(const float* currents, unsigned int size);
	void initMsgs();
};

#endif /* MOTORARRAYROS_H_ */