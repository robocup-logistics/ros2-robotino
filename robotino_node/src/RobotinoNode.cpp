#include "robotino_node/RobotinoNode.hpp"
#include <chrono>

RobotinoNode::RobotinoNode(const std::string& name) 
    : Node(name)
{
    initModules();
}

RobotinoNode::~RobotinoNode() = default;

void RobotinoNode::initModules() 
{
    com_ = std::make_shared<ComROS>();
    com_->setName("RobotinoNode");
    com_->setAddress("192.168.5.90:12080");
    com_->connectToServer(true);

    power_management_ = std::make_shared<PowerManagementROS>(this);
    power_management_->setComId(com_->id());

    omni_drive_ = std::make_shared<OmniDriveROS>(this);
    omni_drive_->setMaxMin(0.5, 0.05, 0.5, 0.05);
}

void RobotinoNode::execute()
{
    com_->processEvents();
    // power_management_.processEvents();
    // auto voltage = power_management_.voltage();
    // auto current = power_management_.current();
    // auto battery_low = power_management_.batteryLow ();
    // RCLCPP_INFO_STREAM(this->get_logger(), "voltage = " << voltage << " , current = " << current << " , battery_low = " << battery_low);
}