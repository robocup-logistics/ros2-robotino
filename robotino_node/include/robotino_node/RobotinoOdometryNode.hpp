#ifndef ROBOTINOODOMETRYNODE_HPP_
#define ROBOTINOODOMETRYNODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "robotino_node/ComROS.hpp"
#include "robotino_node/OdometryROS.hpp"

class RobotinoOdometryNode : public rclcpp::Node
{
public:
	RobotinoOdometryNode(const std::string& name);
	~RobotinoOdometryNode();

	void execute();

private:
	std::shared_ptr<ComROS> com_;
	std::shared_ptr<OdometryROS> odometry_;

	void initModules();
};

#endif /* ROBOTINOODOMETRYNODE_HPP_ */