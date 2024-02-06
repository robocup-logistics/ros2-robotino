//   Author: Saurabh Borse(saurabh.borse@alumni.fh-aachen.de)

//   MIT License
//   Copyright (c) 2023 Saurabh Borse
//   Permission is hereby granted, free of charge, to any person obtaining a copy
//   of this software and associated documentation files (the "Software"), to deal
//   in the Software without restriction, including without limitation the rights
//   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//   copies of the Software, and to permit persons to whom the Software is
//   furnished to do so, subject to the following conditions:
//   The above copyright notice and this permission notice shall be included in all
//   copies or substantial portions of the Software.

//   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//   SOFTWARE.

#ifndef GYROSCOPEROS_HPP_
#define GYROSCOPEROS_HPP_

#include "rec/robotino/api2/GyroscopeExt.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include </usr/include/eigen3/Eigen/Geometry>

class GyroscopeROS: public rec::robotino::api2::GyroscopeExt
{
public:
	GyroscopeROS(rclcpp::Node* node);
	~GyroscopeROS();

    void setMsgFrameId(std::string tf_prefix);

    Eigen::Quaterniond getQuaternionfromEuler(double roll, double pitch, double yaw);

private:
	rclcpp::Node* node_;
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
	rclcpp::TimerBase::SharedPtr timer_;
	
	sensor_msgs::msg::Imu imu_msg_;

	void gyroscopeExtEvent(float angle, float rate);
	//void timerCallback();
};

#endif /* GYROSCOPEROS_HPP_ */