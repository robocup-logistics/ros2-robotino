#ifndef ODOMETRYROS_HPP_
#define ODOMETRYROS_HPP_

#include "rec/robotino/api2/Odometry.h"

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "rto_msgs/srv/reset_odometry.hpp"

class OdometryROS: public rec::robotino::api2::Odometry
{
public:
	OdometryROS(rclcpp::Node* node);
	~OdometryROS();

	void setFrameId(const std::string& tf_prefix);

	void setOdomTransform(bool publish_odom_tf);

private:
	rclcpp::Node* node_;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
	rclcpp::Service<rto_msgs::srv::ResetOdometry>::SharedPtr reset_odometry_server_;
	nav_msgs::msg::Odometry odometry_msg_;
	geometry_msgs::msg::TransformStamped odometry_transform_;
	std::unique_ptr<tf2_ros::TransformBroadcaster> odometry_transform_broadcaster_;
	bool publish_tf=false;

	void readingsEvent(double x, double y, double phi,
		float vx, float vy, float omega, unsigned int sequence);

	void resetOdometryCallback(
		const std::shared_ptr<rto_msgs::srv::ResetOdometry::Request> req,
		std::shared_ptr<rto_msgs::srv::ResetOdometry::Response> res);
};

#endif /* ODOMETRYROS_HPP_ */
