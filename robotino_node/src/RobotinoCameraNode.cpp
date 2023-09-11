#include "robotino_node/RobotinoCameraNode.hpp"

RobotinoCameraNode::RobotinoCameraNode(const std::string& name)
	: Node(name)
{
	this->declare_parameter("hostname", "172.26.1.1");
	this->declare_parameter("cameraNumber", 0);

	initModules();
}

RobotinoCameraNode::~RobotinoCameraNode()
{
}

void RobotinoCameraNode::initModules()
{
	auto hostname = this->get_parameter("hostname").as_string();
	auto camera_number = this->get_parameter("cameraNumber").as_int();

	std::ostringstream os;
	os << "Camera" << camera_number;

    com_ = std::make_shared<ComROS>();
	com_->setName(os.str());
	com_->setAddress(hostname.c_str());

	camera_ = std::make_shared<CameraROS>(this);
	camera_->setComId(com_->id());
	camera_->setNumber(camera_number);

	com_->connectToServer(false);
}

void RobotinoCameraNode::execute()
{
    com_->processEvents();
}