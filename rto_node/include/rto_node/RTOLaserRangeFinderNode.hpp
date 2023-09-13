#ifndef ROBOTINOLASERRANGEFINDERNODE_HPP_
#define ROBOTINOLASERRANGEFINDERNODE_HPP_

#include "rto_node/ComROS.hpp"
#include "rto_node/LaserRangeFinderROS.hpp"

class RTOLaserRangeFinderNode : public rclcpp::Node
{
public:
	RTOLaserRangeFinderNode(const std::string& name);
	~RTOLaserRangeFinderNode();

	void execute();

private:
	std::shared_ptr<ComROS> com_;
	std::shared_ptr<LaserRangeFinderROS> laser_range_finder_;

	void initModules();
};

#endif /* ROBOTINOLASERRANGEFINDERNODE_HPP_ */