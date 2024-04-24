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

#include "rto_node/GyroscopeROS.hpp"
#include <iostream>
#include </usr/include/eigen3/Eigen/Geometry>
#include "sensor_msgs/msg/imu.hpp"

GyroscopeROS::GyroscopeROS(rclcpp::Node* node) : node_(node)
{
	imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
}

GyroscopeROS::~GyroscopeROS()
{
}

void GyroscopeROS::setMsgFrameId(std::string tf_prefix)
{
	imu_msg_.header.frame_id = tf_prefix + "imu_link";
}

Eigen::Quaterniond GyroscopeROS::getQuaternionfromEuler(double roll, double pitch, double yaw)
{
    Eigen::Quaterniond quaternion;
    quaternion = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
               * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
               * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
    return quaternion;
}

void GyroscopeROS::gyroscopeExtEvent(float angle, float rate)
{

    // Populate the IMU message with your data
    imu_msg_.header.stamp = node_->now();

    float roll = 0.0;  // Set your orientation values
    float pitch = 0.0;
    float yaw = angle;

    Eigen::Quaterniond resultQuaternion = getQuaternionfromEuler(roll, pitch, yaw);

    imu_msg_.orientation.x = resultQuaternion.coeffs()[0];
    imu_msg_.orientation.y = resultQuaternion.coeffs()[1];
    imu_msg_.orientation.z = resultQuaternion.coeffs()[2];
    imu_msg_.orientation.w = resultQuaternion.coeffs()[3];
    imu_msg_.angular_velocity.x = 0.0;
    imu_msg_.angular_velocity.y = 0.0;
    imu_msg_.angular_velocity.z = rate;
    imu_msg_.linear_acceleration.x = 0.0;
    imu_msg_.linear_acceleration.y = 0.0;
    imu_msg_.linear_acceleration.z = 9.81;

    imu_pub_->publish(std::move(imu_msg_));

}



