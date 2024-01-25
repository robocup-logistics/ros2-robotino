#include "rto_node/DistanceSensorArrayROS.hpp"
#include <cmath>

DistanceSensorArrayROS::DistanceSensorArrayROS(rclcpp::Node* node) : node_(node)
{
	irpcloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud>("IrSensor_pcl", 10);
	irlaserscan_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud>("IrSensor_scan", 10);
}

DistanceSensorArrayROS::~DistanceSensorArrayROS()
{
}

void DistanceSensorArrayROS::setMsgFrameId(std::string tf_prefix)
{
	irpcloud_msg_.header.frame_id = tf_prefix + "/irpcl_link";
	irlaserscan_msg_.header.frame_id = tf_prefix + "/irscan_link";
}

void DistanceSensorArrayROS::distancesChangedEvent(const float* distances, unsigned int size)
{
	// Build the PointCloud msg
	irpcloud_msg_.header.stamp = node_->now();
	irpcloud_msg_.points.resize(size);

	for(unsigned int i = 0; i < size; ++i)
	{
		// 0.698 radians = 40 Degrees
		// 0.2 is the radius of the robot
		irpcloud_msg_.points[i].x = ( distances[i] + 0.225f ) * cos((M_PI/180)*40 * i);
		irpcloud_msg_.points[i].y = ( distances[i] + 0.225f ) * sin((M_PI/180)*40 * i);
		irpcloud_msg_.points[i].z = 0.05; // 5cm above ground
	}
	// Publish the pcl msg
	irpcloud_pub_->publish(irpcloud_msg_);

	// Build the LaserScan msg
	irlaserscan_msg_.header.stamp = node_->now();
	irlaserscan_msg_.angle_min       = 0;
	irlaserscan_msg_.angle_max       = 2 * M_PI;
	irlaserscan_msg_.angle_increment = (M_PI/180)*40;
	irlaserscan_msg_.scan_time       = 0.1;
	irlaserscan_msg_.range_min       = 0.02;
	//irlaserscan_msg_.range_max       = 0.12;

	irlaserscan_msg_.ranges.resize(size);
	for (uint8_t i = 0; i < 9; ++i) {
		if (irlaserscan_msg_.ranges[i] < 1) {
			irlaserscan_msg_.ranges[i] = distances[i] + 0.225f;
		}
	}

	// Publish the scan msg
	irlaserscan_pub_->publish(irlaserscan_msg_);
}