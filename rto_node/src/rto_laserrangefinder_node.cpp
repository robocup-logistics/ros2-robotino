#include "rclcpp/rclcpp.hpp"

#include "rto_node/RTOLaserRangeFinderNode.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;
  auto node = std::make_shared<RTOLaserRangeFinderNode>("robotino_laserrangefinder_node");

  RCLCPP_INFO(rclcpp::get_logger("rto_lrf_node"), "Starting Robotino Laserrangefinder Node");

  exec.add_node(node);
  auto rate = rclcpp::Rate(30);
  
  while(rclcpp::ok())
  {
    node->execute();
    exec.spin_some();
    rate.sleep();
  }
  
  RCLCPP_INFO(rclcpp::get_logger("rto_lrf_node"), "Shutting down Robotino Laserrangefinder Node");

  rclcpp::shutdown();
  return 0;
}