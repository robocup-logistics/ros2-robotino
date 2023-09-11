#include "robotino_node/CameraROS.hpp"
#include "sensor_msgs/fill_image.hpp"

extern bool sensor_msgs::fillImage(sensor_msgs::msg::Image &image, const std::string &encoding_arg, uint32_t rows_arg, uint32_t cols_arg, uint32_t step_arg, const void *data_arg);

CameraROS::CameraROS(rclcpp::Node* node) : node_(node)
{
}

CameraROS::~CameraROS()
{
}

void CameraROS::setNumber(int number)
{
	std::stringstream img_topic;
	std::stringstream camera_info_topic;

	if( number == 0) 
	{
		img_topic << "image_raw";
		camera_info_topic << "camera_info";
	}
	else 
	{
		img_topic << "image_raw" << number;
		camera_info_topic << "camera_info" << number;
	}

	streaming_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(img_topic.str(), 10);
	camera_info_pub_ = node_->create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic.str(), 10);

	setCameraNumber(number);
}

void CameraROS::imageReceivedEvent(
	const unsigned char* data,
	[[maybe_unused]] unsigned int dataSize,
	unsigned int width,
	unsigned int height,
	unsigned int step )
{
	// Build the Image msg
	auto stamp = node_->now();
	img_msg_.header.stamp = stamp;
	sensor_msgs::fillImage(img_msg_, "rgb8", height, width, step, data);

	// Build the CameraInfo msg
	cam_info_msg_.header.stamp = stamp;
	cam_info_msg_.height = height;
	cam_info_msg_.width = width;

	// Publish the Image & CameraInfo msgs
	streaming_pub_->publish(img_msg_);
	camera_info_pub_->publish(cam_info_msg_);
}