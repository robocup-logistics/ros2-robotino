#include "rclcpp/rclcpp.hpp"

#include "rto_node/RTOOdometryNode.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;
  auto node = std::make_shared<RTOOdometryNode>("robotino_odometry_node");

  exec.add_node(node);
  auto rate = rclcpp::Rate(30);
  
  while(rclcpp::ok())
  {
    node->execute();
    exec.spin_some();
    rate.sleep();
  }
  std::cout << "goodbye" << std::endl;
  rclcpp::shutdown();
  return 0;
}