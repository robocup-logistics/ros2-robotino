#include "robotino_node/RobotinoLaserRangeFinderNode.hpp"
#include <sstream>

RobotinoLaserRangeFinderNode::RobotinoLaserRangeFinderNode(const std::string& name)
	: Node(name)
{
	this->declare_parameter("hostname", "172.26.1.1");
	this->declare_parameter("laserRangeFinderNumber", 0);
	this->declare_parameter("tf_prefix", "no_prefix");

	initModules();
}

RobotinoLaserRangeFinderNode::~RobotinoLaserRangeFinderNode()
{
}

void RobotinoLaserRangeFinderNode::initModules()
{
	std::string hostname = this->get_parameter("hostname").as_string();
	int laserRangeFinderNumber = this->get_parameter("laserRangeFinderNumber").as_int();
	std::string tf_prefix = this->get_parameter("tf_prefix").as_string();

	std::ostringstream os;
	os << "LaserRangeFinder" << laserRangeFinderNumber;

	com_ = std::make_shared<ComROS>();
	com_->setName(os.str());
	com_->setAddress(hostname.c_str());

	laser_range_finder_ = std::make_shared<LaserRangeFinderROS>(this);
	laser_range_finder_->setComId(com_->id());
	laser_range_finder_->setNumber(laserRangeFinderNumber);
	laser_range_finder_->setMsgFrameId(tf_prefix);

	com_->connectToServer(false);
}

void RobotinoLaserRangeFinderNode::execute()
{
	com_->processEvents();
}