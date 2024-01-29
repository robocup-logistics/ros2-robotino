#include "rto_node/MotorArrayROS.hpp"

MotorArrayROS::MotorArrayROS(rclcpp::Node* node) : node_(node)
{
	motor_pub_ = node_->create_publisher<rto_msgs::msg::MotorReadings>("motor_readings", 10);
	joint_states_pub_ = node->create_publisher<sensor_msgs::msg::JointState>(node_->get_namespace()+"/joint_states", 10);

	initMsgs();
}

MotorArrayROS::~MotorArrayROS()
{
}

void MotorArrayROS::initMsgs()
{
    joint_state_msg_.name.resize(3);
	joint_state_msg_.position.resize(3, 0.0);
	joint_state_msg_.velocity.resize(3, 0.0);
	joint_state_msg_.name[0] = "wheel2_joint";
	joint_state_msg_.name[1] = "wheel0_joint";
	joint_state_msg_.name[2] = "wheel1_joint";
}

void MotorArrayROS::getMotorReadings(std::vector<float> &velocities, std::vector<int> &positions)
{
	velocities = motor_msg_.velocities;
	positions = motor_msg_.positions;
}

void MotorArrayROS::velocitiesChangedEvent(const float* velocities, unsigned int size)
{
	// Build the MotorReadings msg
	motor_msg_.velocities.resize(size, 0.0);

	if(velocities != NULL)
	{
		memcpy(motor_msg_.velocities.data(), velocities, size * sizeof(float));
		joint_state_msg_.velocity[0] = ((velocities[2] / 16) * (2 * 3.142) / 60);
		joint_state_msg_.velocity[1] = ((velocities[0] / 16) * (2 * 3.142) / 60);
		joint_state_msg_.velocity[2] = ((velocities[1] / 16) * (2 * 3.142) / 60);
	}
}

void MotorArrayROS::positionsChangedEvent(const int* positions, unsigned int size)
{
	// Build the MotorReadings msg
	motor_msg_.positions.resize(size, 0.0);

	if(positions != NULL)
	{
		memcpy(motor_msg_.positions.data(), positions, size * sizeof(int));
		joint_state_msg_.position[0] = (positions[2] / 16.0 / 2048.0) * (2 * 3.142);
		joint_state_msg_.position[1] = (positions[0] / 16.0 / 2048.0) * (2 * 3.142);
		joint_state_msg_.position[2] = (positions[1] / 16.0 / 2048.0) * (2 * 3.142);
	}
}

void MotorArrayROS::currentsChangedEvent(const float* currents, unsigned int size)
{
	// Build the MotorReadings msg
	motor_msg_.header.stamp = node_->now();
	motor_msg_.currents.resize(size);

	if(currents != NULL)
	{
		memcpy(motor_msg_.currents.data(), currents, size * sizeof(float));
	}
	// Publish the msg
	motor_pub_->publish(motor_msg_);

	joint_state_msg_.header.stamp = node_->now();
	joint_states_pub_->publish(joint_state_msg_);

	
	
}