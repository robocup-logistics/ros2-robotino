#include "rto_node/OmniDriveROS.hpp"

OmniDriveROS::OmniDriveROS(rclcpp::Node* node) : 
	node_(node),
	mGetVelocities{0.0f, 0.0f, 0.0f},
    mGetPositions{0, 0, 0}, 
    motorArray_(node)

{
	cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
		"cmd_vel", 10, std::bind(&OmniDriveROS::cmdVelCallback, this, std::placeholders::_1));
	set_enabled_srv_ = node_->create_service<rto_msgs::srv::SetOmniDriveEnabled>(
		"cmd_vel_enable", std::bind(&OmniDriveROS::handleSetOmniDriveEnabled, this, std::placeholders::_1, std::placeholders::_2));
	set_velocitylimit_srv_ = node_->create_service<rto_msgs::srv::SetVelLimits>(
		"set_velocity_limits", std::bind(&OmniDriveROS::handleSetVelLimits, this, std::placeholders::_1, std::placeholders::_2));
	bumper_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "bumper", 10, std::bind(&OmniDriveROS::bumperCallback, this, std::placeholders::_1));
	motor_error_pub_ = node_->create_publisher<rto_msgs::msg::MotorErrorReadings>("motor_error_readings", 10);
	motor_error_timestamps_.resize(3);
	std::generate(motor_error_timestamps_.begin(), motor_error_timestamps_.end(), 
              [this]() { return node_->now(); });
	motorErrorState_.resize(3);
	std::fill(motorErrorState_.begin(), motorErrorState_.end(), false);

	initMsgs();
}

void OmniDriveROS::initMsgs()
{
	motor_error_msg_.name.resize(3);
	motor_error_msg_.name[0] = "wheel0_joint";
	motor_error_msg_.name[1] = "wheel1_joint";
	motor_error_msg_.name[2] = "wheel2_joint";

	motor_error_msg_.error_status.resize(3);
	std::fill(motor_error_msg_.error_status.begin(), motor_error_msg_.error_status.end(), false);

	motor_error_msg_.error_code.resize(3, 0);
	std::fill(motor_error_msg_.error_code.begin(), motor_error_msg_.error_code.end(), 0);

	motor_error_msg_.error_msg.resize(3, "");
	std::fill(motor_error_msg_.error_msg.begin(), motor_error_msg_.error_msg.end(), "");
}

OmniDriveROS::~OmniDriveROS()
{
}

void OmniDriveROS::bumperCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
	if( msg->data )
	{	
		if (!bumper_hit){
			if (!bumperhit_current_state && !bumperhit_prev_state){
			bumperhit_current_state = true;
			bumper_hit = true;
			timer_->reset();
			}
		}
	}
}

void OmniDriveROS::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{	
	if (!enabled_) {
		RCLCPP_DEBUG(node_->get_logger(), "OmniDrive is disabled. No velocity is set.");
		return;
	}
	if(!bumper_hit){	
		double linear_x = msg->linear.x;
		double linear_y = msg->linear.y;
		double angular = msg->angular.z;
		initMsgs();
		
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
		omniDriveModel_.project(&mSetVelocities[0], &mSetVelocities[1], &mSetVelocities[2], linear_x, linear_y, angular);
		RCLCPP_INFO(node_->get_logger(), "Set velocities: %f, %f, %f", mSetVelocities[0], mSetVelocities[1], mSetVelocities[2]);

		motorArray_.getMotorReadings(mGetVelocities, mGetPositions);
		RCLCPP_INFO(node_->get_logger(), "Get velocities: %f, %f, %f", mGetVelocities[0], mGetVelocities[1], mGetVelocities[2]);

	    rclcpp::Time current_time = node_->now();
	    for (size_t i = 0; i < 3; i++) {
	        if (mSetVelocities[i] == 0.0f) {
	          motorErrorState_[i] = false;
	          motor_error_timestamps_[i] = current_time;
	          continue;
	        }else{
				if (mGetVelocities[i] == 0.0f) {
					if (!motorErrorState_[i]) {
						RCLCPP_INFO(node_->get_logger(), "Trigger receivedd, for motor error");
						motor_error_timestamps_[i] = current_time;
						motorErrorState_[i] = true;
					}else{
						double elapsed_time = (current_time - motor_error_timestamps_[i]).seconds();
						motor_error_msg_.error_status[i] = true;
						motor_error_msg_.error_code[i] = 1;
						motor_error_msg_.error_msg[i] = "[Error]: Set velocity is > 0 but position remains unchanged!";
						if(elapsed_time > motor_timout_){
							enabled_ = false;
							RCLCPP_ERROR(node_->get_logger(), "Sensor_Error for Motor %zu: set velocity > 0 but measured velocity remains 0 for more than %f seconds!", i, motor_timout_);
							motor_error_msg_.error_status[i] = true;
							motor_error_msg_.error_code[i] = 1;
							motor_error_msg_.error_msg[i] = "[ERROR]: Motor " + std::to_string(i) + " disabled due to timeout!";
						}
					}
				}else{
					motor_error_timestamps_[i] = current_time;
				}
	        }
	    }
		motor_error_msg_.header.stamp = node_->now();
		motor_error_pub_->publish(motor_error_msg_);
	}
}

void OmniDriveROS::timerCallback()
{	
	if (bumperhit_current_state && !bumperhit_prev_state){
		bumperhit_prev_state = bumperhit_current_state;
		bumperhit_current_state = false;
		bumper_hit = false;
		timer_->cancel();
		timer_->reset();
	}else if (!bumperhit_current_state && bumperhit_prev_state){
		bumperhit_current_state = false;
		bumperhit_prev_state = false;
		timer_->cancel();
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
	
	motorErrorState_.resize(3);
	std::fill(motorErrorState_.begin(), motorErrorState_.end(), false);
	rclcpp::Time current_time = node_->now();
	for (size_t i = 0; i < 3; i++) {
		motor_error_timestamps_[i] = current_time;
	}
	
    response->success = true;
    RCLCPP_INFO(node_->get_logger(), "OmniDrive is now %s", enabled_ ? "enabled" : "disabled");
    return true;
}

bool OmniDriveROS::handleSetVelLimits(const std::shared_ptr<rto_msgs::srv::SetVelLimits::Request> request,
                                    std::shared_ptr<rto_msgs::srv::SetVelLimits::Response> response)
{	
	max_linear_vel_ = request->max_linear_vel;
	min_linear_vel_ = request->min_linear_vel;
	max_angular_vel_ = request->max_angular_vel;
	min_angular_vel_ = request->min_angular_vel;
    response->success = true;
    RCLCPP_INFO(node_->get_logger(), "Velocity limits set to: max_linear_vel: %f, min_linear_vel: %f, max_angular_vel: %f, min_angular_vel: %f", 
				max_linear_vel_, min_linear_vel_, max_angular_vel_, min_angular_vel_);
    return true;
}

void OmniDriveROS::setBumperTime(double timeout_sec)
{
	bumper_timeout_ = timeout_sec;

	timer_ = node_->create_wall_timer(
        std::chrono::duration<double>(bumper_timeout_), 
        std::bind(&OmniDriveROS::timerCallback, this));
    timer_->cancel(); // Start with the timer canceled
}

void OmniDriveROS::setMotorTimeout(double timeout_sec)
{
	motor_timout_ = timeout_sec;
}
