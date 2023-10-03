#ifndef ROBOTINOSAFETY_HPP_
#define ROBOTINOSAFETY_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "laser_geometry/laser_geometry.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "sensor_msgs/point_cloud2_iterator.hpp"

class RTOSafety : public rclcpp::Node
{
public:
	RTOSafety(const std::string& name);
	~RTOSafety();

private:
	std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>> cmd_vel_pub_;
	std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::Marker>> e1_viz_pub_;
	std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::Marker>> e2_viz_pub_;

	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr rto_cmd_vel_sub_;
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr bumper_sub_;
	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

	geometry_msgs::msg::Twist cmd_vel_msg_;

	visualization_msgs::msg::Marker e1_viz_msg_, e2_viz_msg_;

	laser_geometry::LaserProjection projector_;
	std::shared_ptr<tf2_ros::TransformListener> tfListener_;
	std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

	bool stop_bumper_, stop_laser_, slow_laser_;

	double scale_, dist_;

	// params
	double e1_major_radius_, e1_minor_radius_;
	double e2_major_radius_, e2_minor_radius_;
	int node_loop_rate_;

	void calcScale();
	void buildEllipseVizMsgs();

	void rtoCmdVelCallback(const geometry_msgs::msg::Twist::ConstSharedPtr& msg);
	void bumperCallback(const std_msgs::msg::Bool::ConstSharedPtr& msg);
	void scanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg);

	void check(sensor_msgs::msg::PointCloud2 cloud);

	void inE2(geometry_msgs::msg::Point32 point);
	double solveE1(geometry_msgs::msg::Point32 point);

	void visualizeEllipses();

	void init();
};

#endif /* ROBOTINOSAFETY_HPP_ */