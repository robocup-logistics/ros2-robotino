#include "rclcpp/rclcpp.hpp"

#include "rto_node/RTOOdometryNode.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;
  auto node = std::make_shared<RTOOdometryNode>("robotino_odometry_node");

  RCLCPP_INFO(rclcpp::get_logger("rto_odom_node"), "Starting Robotino Odometry Node");

  exec.add_node(node);
  auto rate = rclcpp::Rate(30);
  
  while(rclcpp::ok())
  {
    node->execute();
    exec.spin_some();
    rate.sleep();
  }
  
  RCLCPP_INFO(rclcpp::get_logger("rto_odom_node"), "Shutting down Robotino Odometry Node");

  rclcpp::shutdown();
  return 0;
}