#include "rto_node/OmniDriveROS.hpp"

OmniDriveROS::OmniDriveROS(rclcpp::Node* node) : node_(node)
{
	cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
		"cmd_vel", 10, std::bind(&OmniDriveROS::cmdVelCallback, this, std::placeholders::_1));
	set_enabled_srv_ = node_->create_service<rto_msgs::srv::SetOmniDriveEnabled>(
		"cmd_vel_enable", std::bind(&OmniDriveROS::handleSetOmniDriveEnabled, this, std::placeholders::_1, std::placeholders::_2));


	bumper_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "bumper", 10, std::bind(&OmniDriveROS::bumperCallback, this, std::placeholders::_1));
}

OmniDriveROS::~OmniDriveROS()
{
}

void OmniDriveROS::bumperCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
	if( msg->data )
	{
		setVelocity(0.0, 0.0, 0.0);
		bumper_hit = true;
		timer_->reset();
	}
}

void OmniDriveROS::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{	
	if (!enabled_) {
		RCLCPP_DEBUG(node_->get_logger(), "OmniDrive is disabled. No velocity is set.");
		return;
	}
	if( !bumper_hit ){	
		double linear_x = msg->linear.x;
		double linear_y = msg->linear.y;
		double angular = msg->angular.z;

		if ( fabs( linear_x ) > max_linear_vel_ )
		{
			if( linear_x > 0.0 )
				linear_x = max_linear_vel_;
			else
				linear_x = -max_linear_vel_;
		}
		else if( fabs( linear_x ) <  min_linear_vel_ && fabs( linear_x ) > 0.0 )
		{
			if( linear_x > 0.0 )
				linear_x = min_linear_vel_;
			else
				linear_x = -min_linear_vel_;
		}

		if ( fabs( linear_y ) > max_linear_vel_ )
		{
			if( linear_y > 0.0 )
				linear_y = max_linear_vel_;
			else
				linear_y = -max_linear_vel_;
		}
		else if( fabs( linear_y ) <  min_linear_vel_ && fabs( linear_y ) > 0.0 )
		{
			if( linear_y > 0.0 )
				linear_y = min_linear_vel_;
			else
				linear_y = -min_linear_vel_;
		}

		if ( fabs( angular ) > max_angular_vel_ )
		{
			if( angular > 0.0 )
				angular = max_angular_vel_;
			else
				angular = -max_angular_vel_;
		}
		else if( fabs( angular ) <  min_angular_vel_ && fabs( angular ) > 0.0 )
		{
			if( angular > 0.0 )
				angular = min_angular_vel_;
			else
				angular = -min_angular_vel_;
		}

		setVelocity(linear_x, linear_y, angular);
	}
}

void OmniDriveROS::timerCallback()
{
    bumper_hit = false;
    timer_->cancel();
}

void OmniDriveROS::setMaxMin(double max_linear_vel, double min_linear_vel, double max_angular_vel, double min_angular_vel)
{
	max_linear_vel_ = max_linear_vel;
	min_linear_vel_ = min_linear_vel;
	max_angular_vel_ = max_angular_vel;
	min_angular_vel_ = min_angular_vel;
}

bool OmniDriveROS::handleSetOmniDriveEnabled(const std::shared_ptr<rto_msgs::srv::SetOmniDriveEnabled::Request> request,
                                    std::shared_ptr<rto_msgs::srv::SetOmniDriveEnabled::Response> response)
{
    enabled_ = request->enable;
    response->success = true;
    RCLCPP_INFO(node_->get_logger(), "OmniDrive is now %s", enabled_ ? "enabled" : "disabled");
    return true;
}



void OmniDriveROS::setBumperTime(double period_sec)
{
	timer_period_ = period_sec;

	timer_ = node->create_wall_timer(
        std::chrono::duration<double>(timer_period_), 
        std::bind(&OmniDriveROS::timerCallback, this));
    timer_->cancel(); // Start with the timer canceled
}