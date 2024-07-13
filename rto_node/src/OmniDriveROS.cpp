#include "rto_node/OmniDriveROS.hpp"

OmniDriveROS::OmniDriveROS(rclcpp::Node* node) : node_(node)
{
	cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
		"cmd_vel", 10, std::bind(&OmniDriveROS::cmdVelCallback, this, std::placeholders::_1));
	set_enabled_srv_ = node_->create_service<rto_msgs::srv::SetOmniDriveEnabled>(
		"cmd_vel_enable", std::bind(&OmniDriveROS::handleSetOmniDriveEnabled, this, std::placeholders::_1, std::placeholders::_2));
	bumper_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "bumper", 10, std::bind(&OmniDriveROS::bumperCallback, this, std::placeholders::_1));
	ir_scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
		"irsensor_scan", 10, std::bind(&OmniDriveROS::irScanCallback, this, std::placeholders::_1));
	velocity_timer_ = node_->create_wall_timer(
		std::chrono::milliseconds(1000 / frequency_), 
		std::bind(&OmniDriveROS::velocityTimerCallback, this));
	velocity_timer_->cancel();

}

OmniDriveROS::~OmniDriveROS()
{
}


void OmniDriveROS::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
	if (!enabled_) {
		RCLCPP_DEBUG(node_->get_logger(), "OmniDrive is disabled. No velocity is set.");
		return;
	}
	if (!bumper_hit){
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

void OmniDriveROS::bumperCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
	if( msg->data )
	{	
		if (!bumper_hit){
			bumper_hit = true;
			RCLCPP_INFO(node_->get_logger(), "Bumper hit detected, setting timer for %f seconds", timer_period_);
		}

	}
}

void OmniDriveROS::setBumperTime(double period_sec)
{
	timer_period_ = period_sec;

	timer_ = node_->create_wall_timer(
        std::chrono::duration<double>(timer_period_), 
        std::bind(&OmniDriveROS::timerCallback, this));
    timer_->cancel(); // Start with the timer canceled

}

void OmniDriveROS::timerCallback()
{	
	bumper_hit = false;
	RCLCPP_INFO(node_->get_logger(), "Timer expired, bumper hit flag reset");
}

void OmniDriveROS::irScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    if (bumper_hit)
    {	
		if (!sensor_check_){
			sensor_check_ = true;
			RCLCPP_INFO(node_->get_logger(), "Bumper hit detected, initiating recovery");
			bool path_found = false;
			for (size_t i = 0; i < msg->ranges.size() - 1; i++){
				if (msg->ranges[i] > 0.5 && msg->ranges[i + 1] > 0.5){	
					path_found = true;
					theta_ = msg->angle_min + (i + (i + 1)) / 2.0 * msg->angle_increment;
					x_ = max_linear_vel_ * cos(theta_);
					y_ = max_linear_vel_ * sin(theta_);
					RCLCPP_INFO(node_->get_logger(), "Clear path detected at %f degrees", theta_);
					velocity_end_time_ = node_->now() + rclcpp::Duration(velocity_duration_, 0);
					velocity_timer_->reset();
					
					break; 
				}
			}

			if (!path_found)
			{
				RCLCPP_INFO(node_->get_logger(), "No clear path detected, recovery failed");
				RCLCPP_INFO(node_->get_logger(), "Initiating timer for %f seconds", timer_period_);
				timer_->reset();
			}
		}
    }
}

void OmniDriveROS::velocityTimerCallback()
{
    if (node_->now() < velocity_end_time_){
		RCLCPP_INFO(node_->get_logger(), "SEtting velocity commands for %f seconds at freqency %d", velocity_duration_, frequency_);
        setVelocity(x_, y_, 0.0);
    }else{
        velocity_timer_->cancel();
        RCLCPP_INFO(node_->get_logger(), "Stopping velocity commands after set duration");
		bumper_hit = false;
		sensor_check_ = false;
    }
}