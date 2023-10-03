#include "rclcpp/rclcpp.hpp"

#include "rto_node/RTOCameraNode.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;
  auto node = std::make_shared<RTOCameraNode>("robotino_camera_node");

  RCLCPP_INFO(rclcpp::get_logger("rto_camera_node"), "Starting Robotino Camera Node");

  exec.add_node(node);
  auto rate = rclcpp::Rate(30);
  
  while(rclcpp::ok())
  {
    node->execute();
    exec.spin_some();
    rate.sleep();
  }
  
  RCLCPP_INFO(rclcpp::get_logger("rto_camera_node"), "Shutting down Robotino Camera Node");

  rclcpp::shutdown();
  return 0;
}