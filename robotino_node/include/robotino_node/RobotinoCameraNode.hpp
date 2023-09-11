#ifndef ROBOTINOCAMERANODE_HPP_
#define ROBOTINOCAMERANODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "robotino_node/ComROS.hpp"
#include "robotino_node/CameraROS.hpp"

class RobotinoCameraNode : public rclcpp::Node
{
public:
	RobotinoCameraNode(const std::string& name);
	~RobotinoCameraNode();

	void execute();

private:
	std::shared_ptr<ComROS> com_;
	std::shared_ptr<CameraROS> camera_;

	void initModules();
};

#endif /* ROBOTINOCAMERANODE_HPP_ */