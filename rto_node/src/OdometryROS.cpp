#include "rto_node/OdometryROS.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

OdometryROS::OdometryROS(rclcpp::Node* node) : node_(node)
{
	odometry_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
	reset_odometry_server_ = node_->create_service<rto_msgs::srv::ResetOdometry>("reset_odometry", std::bind(&OdometryROS::resetOdometryCallback, this, std::placeholders::_1, std::placeholders::_2));
	odometry_transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);
}

OdometryROS::~OdometryROS()
{
}

void OdometryROS::setFrameId(const std::string& tf_prefix)
{
	odometry_msg_.header.frame_id = (tf_prefix == "") ? "odom" : (tf_prefix + "odom");
	odometry_msg_.child_frame_id = (tf_prefix == "") ? "base_link" : (tf_prefix + "base_link");
	odometry_transform_.header.frame_id = (tf_prefix == "") ? "odom" : (tf_prefix + "odom");
	odometry_transform_.child_frame_id = (tf_prefix == "") ? "base_link" : (tf_prefix + "base_link");
}

void OdometryROS::readingsEvent(double x, double y, double phi,
	float vx, float vy, float omega, [[maybe_unused]] unsigned int sequence )
{
	tf2::Quaternion phi_quat;
	phi_quat.setRPY(0, 0, phi);

	// Construct messages
	odometry_msg_.header.stamp = node_->now();
	odometry_msg_.pose.pose.position.x = x;
	odometry_msg_.pose.pose.position.y = y ;
	odometry_msg_.pose.pose.position.z = 0.0;
	odometry_msg_.pose.pose.orientation.x = phi_quat.getX();
	odometry_msg_.pose.pose.orientation.y = phi_quat.getY();
	odometry_msg_.pose.pose.orientation.z = phi_quat.getZ();
	odometry_msg_.pose.pose.orientation.w = phi_quat.getW();
	odometry_msg_.twist.twist.linear.x = vx;
	odometry_msg_.twist.twist.linear.y = vy;
	odometry_msg_.twist.twist.linear.z = 0.0;
	odometry_msg_.twist.twist.angular.x = 0.0;
	odometry_msg_.twist.twist.angular.y = 0.0;
	odometry_msg_.twist.twist.angular.z = omega;

	// Commentd: Transform is not needed, being published by the ekf fusion node
	// odometry_transform_.header.stamp = odometry_msg_.header.stamp;
	// odometry_transform_.transform.translation.x = x;
	// odometry_transform_.transform.translation.y = y;
	// odometry_transform_.transform.translation.z = 0.0;
	// odometry_transform_.transform.rotation.x = phi_quat.getX();
	// odometry_transform_.transform.rotation.y = phi_quat.getY();
	// odometry_transform_.transform.rotation.z = phi_quat.getZ();
	// odometry_transform_.transform.rotation.w = phi_quat.getW();

	// odometry_transform_broadcaster_->sendTransform(odometry_transform_);

	// Publish the msg
	odometry_pub_->publish(odometry_msg_);
}

void OdometryROS::resetOdometryCallback(
	const std::shared_ptr<rto_msgs::srv::ResetOdometry::Request> req,
	[[maybe_unused]] std::shared_ptr<rto_msgs::srv::ResetOdometry::Response> res)
{
	set(req->x, req->y, req->phi, true);
}