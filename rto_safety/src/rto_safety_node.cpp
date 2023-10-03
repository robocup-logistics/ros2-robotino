#include "rclcpp/rclcpp.hpp"

#include "rto_safety/RTOSafetyNode.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;
  auto node = std::make_shared<RTOSafety>("robotino_safety_node");

  RCLCPP_INFO(rclcpp::get_logger("rto_safety_node"), "Starting Robotino Safety Node");

  exec.add_node(node);
  auto rate = rclcpp::Rate(20);
  
  while(rclcpp::ok())
  {
    exec.spin_some();
    rate.sleep();
  }
  
  RCLCPP_INFO(rclcpp::get_logger("rto_safety_node"), "Shutting down Robotino Safety Node");

  rclcpp::shutdown();
  return 0;
}