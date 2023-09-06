/*
 * RobotinoNode.cpp
 *
 *  Created on: 09.12.2011
 *      Author: indorewala@servicerobotics.eu
 */

#include "robotino_node/RobotinoOdometryNode.hpp"

RobotinoOdometryNode::RobotinoOdometryNode(const std::string& name)
	: Node(name)
{
	//nh_.param<std::string>("hostname", hostname_, "192.168.5.5" );
	//nh_.param<std::string>("tf_prefix", tf_prefix, "no_prefix");


	initModules();
}

RobotinoOdometryNode::~RobotinoOdometryNode() = default;

void RobotinoOdometryNode::initModules()
{
    com_ = std::make_shared<ComROS>();
	com_->setName("Odometry");
	com_->setAddress("192.168.5.90:12080");

	odometry_ = std::make_shared<OdometryROS>(this);

	// Set the ComIds
	odometry_->setComId(com_->id());

	// Set frame Id
    auto tf_prefix = "no_prefix";
	odometry_->setFrameId(tf_prefix);

	com_->connectToServer(false);
}

void RobotinoOdometryNode::execute()
{
    com_->processEvents();
}