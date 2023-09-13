#ifndef LASERRANGEFINDERROS_HPP_
#define LASERRANGEFINDERROS_HPP_

#include "rec/robotino/api2/LaserRangeFinder.h"
#include "rec/robotino/api2/LaserRangeFinderReadings.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LaserRangeFinderROS: public rec::robotino::api2::LaserRangeFinder
{
public:
	LaserRangeFinderROS(rclcpp::Node* node);
	~LaserRangeFinderROS();

	void setNumber(int number);
	void setMsgFrameId(std::string tf_prefix);

private:
	rclcpp::Node* node_;
	rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_pub_;
	sensor_msgs::msg::LaserScan laser_scan_msg_;

	void scanEvent(const rec::robotino::api2::LaserRangeFinderReadings &scan);
};

#endif /* LASERRANGEFINDERROS_HPP_ */