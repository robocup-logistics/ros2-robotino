#ifndef ROBOTINOCAMERANODE_HPP_
#define ROBOTINOCAMERANODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rto_node/ComROS.hpp"
#include "rto_node/CameraROS.hpp"

class RTOCameraNode : public rclcpp::Node
{
public:
	RTOCameraNode(const std::string& name);
	~RTOCameraNode();

	void execute();

private:
	std::shared_ptr<ComROS> com_;
	std::shared_ptr<CameraROS> camera_;

	void initModules();
};

#endif /* ROBOTINOCAMERANODE_HPP_ */