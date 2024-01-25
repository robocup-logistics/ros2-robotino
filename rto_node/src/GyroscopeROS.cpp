#include "rto_node/GyroscopeROS.hpp"
#include <iostream>
#include <Eigen/Geometry>

GyroscopeROS::GyroscopeROS(rclcpp::Node* node) : node_(node)
{
	imu_pub_ = node_->create_publisher<sensor_msgs::msg::imu>("imu", 10);
}

GyroscopeROS::~GyroscopeROS()
{
}

void GyroscopeROS::setMsgFrameId(std::string tf_prefix)
{
	irpcloud_msg_.header.frame_id = tf_prefix + "/irpcl_link";
}

void GyroscopeROS::getQuaternionfromEuler(double roll, double pitch, double yaw); 
{
    Eigen::Quaterniond quaternion;
    quaternion = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
               * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
               * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
    return quaternion;
}

void GyroscopeROS::gyroscopeEvent(float angle, float rate)
{   

    // Populate the IMU message with your data
    imu_msg_->header.stamp = node_->now();
    imu_msg_->header.frame_id = "base_link";  // Set the appropriate frame_id

    double roll = 0.0;  // Set your orientation values
    double pitch = 0.0;
    double yaw = angle

    Eigen::Quaterniond resultQuaternion = getQuaternionfromEuler(roll, pitch, yaw);

    imu_msg_->orientation.x = resultQuaternion.coeffs()[0];  
    imu_msg_->orientation.y = resultQuaternion.coeffs()[1];
    imu_msg_->orientation.z = resultQuaternion.coeffs()[2];
    imu_msg_->orientation.w = resultQuaternion.coeffs()[3];
    imu_msg_->angular_velocity.x = 0.0;  
    imu_msg_->angular_velocity.y = 0.0;
    imu_msg_->angular_velocity.z = 0.0;
    imu_msg_->linear_acceleration.x = 0.0;  
    imu_msg_->linear_acceleration.y = 0.0;
    imu_msg_->linear_acceleration.z = 9.81; 

    imu_pub_->publish(std::move(imu_msg_));

}


