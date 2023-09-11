#ifndef DISTANCESENSORARRAYROS_HPP_
#define DISTANCESENSORARRAYROS_HPP_

#include "rec/robotino/api2/DistanceSensorArray.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"

class DistanceSensorArrayROS: public rec::robotino::api2::DistanceSensorArray
{
public:
	DistanceSensorArrayROS(rclcpp::Node* node);
	~DistanceSensorArrayROS();

	void setMsgFrameId(std::string tf_prefix);

private:
	rclcpp::Node* node_;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr distances_pub_;
	sensor_msgs::msg::PointCloud distances_msg_;

	void distancesChangedEvent(const float* distances, unsigned int size);
};


#endif /* DISTANCESENSORARRAYROS_HPP_ */