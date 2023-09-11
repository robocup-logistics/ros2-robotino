#ifndef ROBOTINOLASERRANGEFINDERNODE_HPP_
#define ROBOTINOLASERRANGEFINDERNODE_HPP_

#include "robotino_node/ComROS.hpp"
#include "robotino_node/LaserRangeFinderROS.hpp"

class RobotinoLaserRangeFinderNode : public rclcpp::Node
{
public:
	RobotinoLaserRangeFinderNode(const std::string& name);
	~RobotinoLaserRangeFinderNode();

	void execute();

private:
	std::shared_ptr<ComROS> com_;
	std::shared_ptr<LaserRangeFinderROS> laser_range_finder_;

	void initModules();
};

#endif /* ROBOTINOLASERRANGEFINDERNODE_HPP_ */