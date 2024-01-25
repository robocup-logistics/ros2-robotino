#ifndef GYROSCOPEROS_HPP_
#define GYROSCOPEROS_HPP_

#include "rec/robotino/api2/Gyroscope.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class GyroscopeROS: public rec::robotino::api2::Gyroscope
{
public:
	GyroscopeROS(rclcpp::Node* node);
	~GyroscopeROS();

    void setMsgFrameId(std::string tf_prefix);

    void getQuaternionfromEuler(double roll, double pitch, double yaw);

private:
	rclcpp::Node* node_;
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
	
	sensor_msgs::msg::Imu imu_msg_;

	void gyroscopeEvent(float angle, float rate);
};

#endif /* GYROSCOPEROS_HPP_ */