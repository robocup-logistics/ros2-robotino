#ifndef DISTANCESENSORARRAYROS_HPP_
#define DISTANCESENSORARRAYROS_HPP_

#include "rec/robotino/api2/DistanceSensorArray.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>

class DistanceSensorArrayROS: public rec::robotino::api2::DistanceSensorArray
{
public:
	DistanceSensorArrayROS(rclcpp::Node* node);
	~DistanceSensorArrayROS();

	void setMsgFrameId(std::string tf_prefix);

private:
	rclcpp::Node* node_;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr irpcloud_pub_;
	sensor_msgs::msg::PointCloud irpcloud_msg_;

	rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr irlaserscan_pub_;
	sensor_msgs::msg::LaserScan irlaserscan_msg_;

	void distancesChangedEvent(const float* distances, unsigned int size);
};

#endif /* DISTANCESENSORARRAYROS_HPP_ */