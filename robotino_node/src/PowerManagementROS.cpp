#include "robotino_node/PowerManagementROS.hpp"


PowerManagementROS::PowerManagementROS(rclcpp::Node* node) : node_(node)
{
    publisher_ = node_->create_publisher<robotino_msgs::msg::PowerReadings>("battery_status", 10);
    timer_ = node_->create_wall_timer(std::chrono::milliseconds(500), std::bind(&PowerManagementROS::powerTimerCb, this));
}

PowerManagementROS::~PowerManagementROS() = default;

// Does not work - major api fuckup
void PowerManagementROS::readingsEvent([[maybe_unused]] float battery_voltage, [[maybe_unused]] float system_current)
{
    //std::cout << "power: " << system_current << ", " << battery_voltage << std::endl;
}

// Workaround for broken readingsEvent
void PowerManagementROS::powerTimerCb()
{
    auto msg = robotino_msgs::msg::PowerReadings();
    msg.voltage = this->voltage();
    msg.current = this->current();
    msg.battery_low = this->batteryLow();

    publisher_->publish(msg);
}