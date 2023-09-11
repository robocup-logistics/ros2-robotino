#include "robotino_node/DistanceSensorArrayROS.hpp"
#include <cmath>

DistanceSensorArrayROS::DistanceSensorArrayROS(rclcpp::Node* node) : node_(node)
{
	distances_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud>("distance_sensors", 10);
}

DistanceSensorArrayROS::~DistanceSensorArrayROS()
{
}

void DistanceSensorArrayROS::setMsgFrameId(std::string tf_prefix)
{
	distances_msg_.header.frame_id = (tf_prefix == "no_prefix") ? "base_link" : (tf_prefix + "/base_link");
}

void DistanceSensorArrayROS::distancesChangedEvent(const float* distances, unsigned int size)
{
	// Build the PointCloud msg
	distances_msg_.header.stamp = node_->now();
	distances_msg_.points.resize(size);

	for(unsigned int i = 0; i < size; ++i)
	{
		// 0.698 radians = 40 Degrees
		// 0.2 is the radius of the robot
		distances_msg_.points[i].x = ( distances[i] + 0.2 ) * cos(0.698 * i);
		distances_msg_.points[i].y = ( distances[i] + 0.2 ) * sin(0.698 * i);
		distances_msg_.points[i].z = 0.05; // 5cm above ground
	}

	// Publish the msg
	distances_pub_->publish(distances_msg_);
}