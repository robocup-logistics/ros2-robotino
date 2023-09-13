#ifndef ROBOTINOODOMETRYNODE_HPP_
#define ROBOTINOODOMETRYNODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rto_node/ComROS.hpp"
#include "rto_node/OdometryROS.hpp"

class RTOOdometryNode : public rclcpp::Node
{
public:
	RTOOdometryNode(const std::string& name);
	~RTOOdometryNode();

	void execute();

private:
	std::shared_ptr<ComROS> com_;
	std::shared_ptr<OdometryROS> odometry_;

	void initModules();
};

#endif /* ROBOTINOODOMETRYNODE_HPP_ */