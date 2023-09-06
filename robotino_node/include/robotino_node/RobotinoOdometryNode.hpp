/*
 * RobotinoNode.h
 *
 *  Created on: 09.12.2011
 *      Author: indorewala@servicerobotics.eu
 */

#ifndef ROBOTINOODOMETRYNODE_H_
#define ROBOTINOODOMETRYNODE_H_

#include "rclcpp/rclcpp.hpp"
#include "robotino_node/ComROS.hpp"
#include "robotino_node/OdometryROS.hpp"

class RobotinoOdometryNode : public rclcpp::Node
{
public:
	RobotinoOdometryNode(const std::string& name);
	virtual ~RobotinoOdometryNode();

	void execute();

private:
	std::string hostname_;
	std::string tf_prefix;

	std::shared_ptr<ComROS> com_;
	std::shared_ptr<OdometryROS> odometry_;

	void initModules();
};

#endif /* ROBOTINOODOMETRYNODE_H_ */