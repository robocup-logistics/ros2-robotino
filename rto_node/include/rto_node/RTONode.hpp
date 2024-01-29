#ifndef ROBOTINONODE_HPP_
#define ROBOTINONODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rto_node/ComROS.hpp"
#include "rto_node/BumperROS.hpp"
#include "rto_node/MotorArrayROS.hpp"
#include "rto_node/PowerManagementROS.hpp"
#include "rto_node/OmniDriveROS.hpp"
#include "rto_node/GyroscopeROS.hpp"
#include "rto_node/AnalogInputArrayROS.hpp"
#include "rto_node/DigitalInputArrayROS.hpp"
#include "rto_node/DigitalOutputArrayROS.hpp"
#include "rto_node/DistanceSensorArrayROS.hpp"
#include "rto_node/ElectricalGripperROS.hpp"
#include "rto_node/EncoderInputROS.hpp"

//#include "sensor_msgs/msg/joint_state.hpp"

class RTONode : public rclcpp::Node
{
public:
	RTONode(const std::string& name);
	~RTONode();

	void execute();

private:
	//std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>> joint_states_pub_;

	//sensor_msgs::msg::JointState joint_state_msg_;
	//std::vector<float> motor_velocities_;
	//std::vector<int> motor_positions_;

    std::shared_ptr<ComROS> com_;
	std::shared_ptr<BumperROS> bumper_;
	std::shared_ptr<MotorArrayROS> motor_array_;
	std::shared_ptr<PowerManagementROS> power_management_;
	std::shared_ptr<OmniDriveROS> omni_drive_;
	std::shared_ptr<GyroscopeROS> imu_;
	std::shared_ptr<AnalogInputArrayROS> analog_input_array_;
	std::shared_ptr<DigitalInputArrayROS> digital_input_array_;
	std::shared_ptr<DigitalOutputArrayROS> digital_output_array_;
	std::shared_ptr<DistanceSensorArrayROS> distance_sensor_array_;
	std::shared_ptr<ElectricalGripperROS> electrical_gripper_;
	std::shared_ptr<EncoderInputROS> encoder_input_;

	void initModules();
	//void initMsgs();
	//void publishJointStateMsg();
};

#endif /* ROBOTINONODE_HPP_ */