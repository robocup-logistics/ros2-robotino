#ifndef CAMERAROS_HPP_
#define CAMERAROS_HPP_

#include "rec/robotino/api2/Camera.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

class CameraROS : public rec::robotino::api2::Camera
{
public:
	CameraROS(rclcpp::Node* node);
	~CameraROS();

	void setNumber( int number );

private:
	rclcpp::Node* node_;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr streaming_pub_;
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
	sensor_msgs::msg::Image img_msg_;
	sensor_msgs::msg::CameraInfo cam_info_msg_;

	void imageReceivedEvent(
		const unsigned char* data,
		unsigned int dataSize,
		unsigned int width,
		unsigned int height,
		unsigned int step );
};

#endif /* CAMERAROS_HPP_ */