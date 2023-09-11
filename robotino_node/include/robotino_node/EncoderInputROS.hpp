#ifndef ENCODERINPUTROS_HPP_
#define ENCODERINPUTROS_HPP_

#include "rec/robotino/api2/EncoderInput.h"

#include "rclcpp/rclcpp.hpp"
#include "robotino_msgs/msg/encoder_readings.hpp"
#include "robotino_msgs/srv/set_encoder_position.hpp"

class EncoderInputROS: public rec::robotino::api2::EncoderInput
{
public:
	EncoderInputROS(rclcpp::Node* node);
	~EncoderInputROS();

private:
	rclcpp::Node* node_;
	rclcpp::Publisher<robotino_msgs::msg::EncoderReadings>::SharedPtr encoder_pub_;
	rclcpp::Service<robotino_msgs::srv::SetEncoderPosition>::SharedPtr encoder_position_server_;
	robotino_msgs::msg::EncoderReadings encoder_msg_;

	void readingsChangedEvent(int velocity, int position, float current);

	void setEncoderPositionCallback(
		const std::shared_ptr<robotino_msgs::srv::SetEncoderPosition::Request> req,
		std::shared_ptr<robotino_msgs::srv::SetEncoderPosition::Response> res);
};

#endif /* ENCODERINPUTROS_HPP_ */