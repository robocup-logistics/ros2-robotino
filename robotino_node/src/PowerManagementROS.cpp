#include "robotino_node/PowerManagementROS.hpp"

PowerManagementROS::PowerManagementROS(rclcpp::Node* node) : node_(node)
{
}

PowerManagementROS::~PowerManagementROS() = default;

// Does not work - major api fuckup
void PowerManagementROS::readingsEvent(float battery_voltage, float system_current)
{
    //std::cout << "power: " << system_current << ", " << battery_voltage << std::endl;
}

