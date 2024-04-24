#include "rto_node/RTOOdometryNode.hpp"

RTOOdometryNode::RTOOdometryNode(const std::string& name)
	: Node(name)
{
	this->declare_parameter("hostname", "172.26.1.1");
	this->declare_parameter("tf_prefix", "");

	initModules();
}

RTOOdometryNode::~RTOOdometryNode()
{
}

void RTOOdometryNode::initModules()
{
	auto hostname = this->get_parameter("hostname").as_string();
	auto tf_prefix = this->get_parameter("tf_prefix").as_string();

    com_ = std::make_shared<ComROS>();
	com_->setName("Odometry");
	com_->setAddress(hostname.c_str());

	odometry_ = std::make_shared<OdometryROS>(this);
	odometry_->setComId(com_->id());
	odometry_->setFrameId(tf_prefix);

	com_->connectToServer(false);
}

void RTOOdometryNode::execute()
{
    com_->processEvents();
}