/*
 * RobotinoNode.h
 *
 *  Created on: 09.12.2011
 *      Author: indorewala@servicerobotics.eu
 */

#ifndef ROBOTINONODE_H_
#define ROBOTINONODE_H_

#include "rclcpp/rclcpp.hpp"
#include "robotino_node/ComROS.hpp"
#include "robotino_node/PowerManagementROS.hpp"
#include "robotino_node/OmniDriveROS.hpp"

class RobotinoNode : public rclcpp::Node
{
public:
	RobotinoNode(const std::string& name);
	virtual ~RobotinoNode();

	void initModules();

	void execute();

private:
    std::shared_ptr<ComROS> com_;
	std::shared_ptr<PowerManagementROS> power_management_;
	std::shared_ptr<OmniDriveROS> omni_drive_;
};

#endif /* ROBOTINONODE_H_ */