#include "rto_node/RTONode.hpp"
#include <chrono>

RTONode::RTONode(const std::string& name)
    : Node(name)
{
    this->declare_parameter("hostname", "172.26.1.1");
    this->declare_parameter("max_linear_vel", 3.0);
    this->declare_parameter("min_linear_vel", 0.02);
    this->declare_parameter("max_angular_vel", 3.0);
    this->declare_parameter("min_angular_vel", 0.07);
    this->declare_parameter("tf_prefix", "");
    this->declare_parameter("bumper_stop_time", 2.0);

    initModules();
}

RTONode::~RTONode()
{
}

void RTONode::initModules()
{
    auto hostname = this->get_parameter("hostname").as_string();
    auto max_linear_vel = this->get_parameter("max_linear_vel").as_double();
    auto min_linear_vel = this->get_parameter("min_linear_vel").as_double();
    auto max_angular_vel = this->get_parameter("max_angular_vel").as_double();
    auto min_angular_vel = this->get_parameter("min_angular_vel").as_double();
    auto tf_prefix = this->get_parameter("tf_prefix").as_string();
    auto bumper_stop_time = this->get_parameter("bumper_stop_time").as_double();

    com_ = std::make_shared<ComROS>();
    com_->setName("RobotinoNode");
    com_->setAddress(hostname.c_str());
    com_->connectToServer(true);

    bumper_ = std::make_shared<BumperROS>(this);
    bumper_->setComId(com_->id());

    motor_array_ = std::make_shared<MotorArrayROS>(this);
    motor_array_->setComId(com_->id());

    power_management_ = std::make_shared<PowerManagementROS>(this);
    power_management_->setComId(com_->id());

    omni_drive_ = std::make_shared<OmniDriveROS>(this);
    omni_drive_->setComId(com_->id());
    omni_drive_->setMaxMin(max_linear_vel, min_linear_vel, max_angular_vel, min_angular_vel);
    omni_drive_->setBumperTime(bumper_stop_time);

    imu_ = std::make_shared<GyroscopeROS>(this);
    imu_->setComId(com_->id());
    imu_->setMsgFrameId(tf_prefix);

    analog_input_array_ = std::make_shared<AnalogInputArrayROS>(this);
    analog_input_array_->setComId(com_->id());

    digital_input_array_ = std::make_shared<DigitalInputArrayROS>(this);
    digital_input_array_->setComId(com_->id());

    digital_output_array_ = std::make_shared<DigitalOutputArrayROS>(this);
    digital_output_array_->setComId(com_->id());

    distance_sensor_array_ = std::make_shared<DistanceSensorArrayROS>(this);
    distance_sensor_array_->setComId(com_->id());
    distance_sensor_array_->setMsgFrameId(tf_prefix);

	electrical_gripper_ = std::make_shared<ElectricalGripperROS>(this);
    electrical_gripper_->setComId(com_->id());

    encoder_input_ = std::make_shared<EncoderInputROS>(this);
    encoder_input_->setComId(com_->id());
}

void RTONode::execute()
{
    com_->processEvents();
}
