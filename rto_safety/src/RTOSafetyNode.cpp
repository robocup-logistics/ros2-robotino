#include "rto_safety/RTOSafetyNode.hpp"
#include <chrono>

RTOSafety::RTOSafety(const std::string& name) :
    Node(name), 
    stop_bumper_(false),
    slow_laser_(false)
{
    this->declare_parameter("outer_major_radius", 0.70);
    this->declare_parameter("outer_minor_radius", 0.30);
    this->declare_parameter("inner_major_radius", 0.40);
    this->declare_parameter("inner_minor_radius", 0.25);

    init();
}

RTOSafety::~RTOSafety()
{
}

void RTOSafety::init()
{
    e2_major_radius_ = this->get_parameter("outer_major_radius").as_double();
    e2_minor_radius_ = this->get_parameter("outer_minor_radius").as_double();
    e1_major_radius_ = this->get_parameter("inner_major_radius").as_double();
    e1_minor_radius_ = this->get_parameter("inner_minor_radius").as_double();

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
	e1_viz_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("inner_ellipse_marker", 10);
	e2_viz_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("outer_ellipse_marker", 10);

    rto_cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/rto_cmd_vel", 10, std::bind(&RTOSafety::rtoCmdVelCallback, this, std::placeholders::_1));
	bumper_sub_ = this->create_subscription<std_msgs::msg::Bool>("/bumper", 10, std::bind(&RTOSafety::bumperCallback, this, std::placeholders::_1));
	scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&RTOSafety::scanCallback, this, std::placeholders::_1));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}


void RTOSafety::calcScale()
{
	geometry_msgs::msg::Point32 point_on_e2;

	point_on_e2.x = e2_major_radius_;
	point_on_e2.y = 0;

	scale_ = solveE1(point_on_e2);
	dist_ = scale_;
}

void RTOSafety::buildEllipseVizMsgs()
{
	e1_viz_msg_.header.frame_id = e2_viz_msg_.header.frame_id = "/base_link";
	e1_viz_msg_.header.stamp = e2_viz_msg_.header.stamp = this->now();
	e1_viz_msg_.ns = "inner_ellipse";
	e2_viz_msg_.ns = "outer_ellipse";

	e1_viz_msg_.action = e2_viz_msg_.action = visualization_msgs::msg::Marker::ADD;
	e1_viz_msg_.type = e2_viz_msg_.type = visualization_msgs::msg::Marker::POINTS;

	// Color the ellipses green
	e1_viz_msg_.color.g = 1.0;
	e1_viz_msg_.color.a = 1.0;
	e2_viz_msg_.color.g = 1.0;
	e2_viz_msg_.color.a = 1.0;

	// Scale the points
	e1_viz_msg_.scale.x = 0.02;
	e1_viz_msg_.scale.y = 0.02;

	e2_viz_msg_.scale.x = 0.02;
	e2_viz_msg_.scale.y = 0.02;

	// Now we populate the msgs
	for(double t = -M_PI/2; t <= M_PI/2; t += 0.1)
	{
		geometry_msgs::msg::Point e1_p, e2_p;

		e1_p.x = e1_major_radius_ * cos(t);
		e1_p.y = e1_minor_radius_ * sin(t);

		e2_p.x = e2_major_radius_ * cos(t);
		e2_p.y = e2_minor_radius_ * sin(t);

		e1_viz_msg_.points.push_back(e1_p);
		e2_viz_msg_.points.push_back(e2_p);
	}
}

void RTOSafety::rtoCmdVelCallback(const geometry_msgs::msg::Twist::ConstSharedPtr& msg)
{
	cmd_vel_msg_.linear.x = (dist_ / scale_) * msg->linear.x;
	cmd_vel_msg_.linear.y = (dist_ / scale_) * msg->linear.y;
	cmd_vel_msg_.angular.z = (dist_ / scale_) * msg->angular.z;

	cmd_vel_pub_->publish(cmd_vel_msg_);
}

void RTOSafety::bumperCallback(const std_msgs::msg::Bool::ConstSharedPtr& msg)
{
	if(msg->data)
	{
        // RCLCPP_ERROR_STREAM("Bumper hit! Shutting down node!");
        rclcpp::shutdown();
	}
}

void RTOSafety::scanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg)
{
	sensor_msgs::msg::PointCloud2 cloud;

	try
	{
        rclcpp::Time stamp = msg->header.stamp;
        auto time_seconds = stamp.seconds() + msg->ranges.size() * msg->time_increment;
        rclcpp::Time time(static_cast<uint64_t>(time_seconds * 1e9));
        tf_buffer_->waitForTransform("/base_link",
            msg->header.frame_id,
            time,
            rclcpp::Duration::from_seconds(1.0),
            nullptr);
		// tfListener_->waitForTransform("/base_link",
        //         msg->header.frame_id,
        //         msg->header.stamp + rclcpp::Duration().fromSec(msg->ranges.size() * msg->time_increment),
        //         rclcpp::Duration(1.0));
		projector_.transformLaserScanToPointCloud("/base_link", *msg, cloud, *tf_buffer_);
	}
	catch(tf2::LookupException& ex)
	{
		// ROS_WARN("Lookup exception: %s\n", ex.what());
		return;
	}
	catch(tf2::ConnectivityException& ex)
	{
		// ROS_WARN("Connectivity exception: %s\n", ex.what());
		return;
	}
	catch(tf2::ExtrapolationException& ex)
	{
		// ROS_WARN("Extrapolation exception: %s\n", ex.what());
		return;
	}
	check(cloud);
	visualizeEllipses();
}

void RTOSafety::check(sensor_msgs::msg::PointCloud2 cloud)
{
	stop_laser_ = false;
	slow_laser_ = false;
	dist_ = scale_;

	for(sensor_msgs::PointCloud2ConstIterator<float> it(cloud, "x"); it != it.end(); ++it)
	{
        geometry_msgs::msg::Point32 p;
        p.x = it[0];
        p.y = it[1];
		inE2(p);
	}
}

void RTOSafety::inE2(geometry_msgs::msg::Point32 point)
{
	double check = pow((point.x / e2_major_radius_), 2) + pow((point.y / e2_minor_radius_), 2) - 1;

	if(check <= 0.0) // Check if the point is in Ellipse 2
	{
		slow_laser_ = true;

		dist_ = solveE1(point);
		if(dist_ <= 0.0 )
		{
			dist_ = 0.0;
			stop_laser_ = true;
		}
	}
}

double RTOSafety::solveE1(geometry_msgs::msg::Point32 point)
{
	return (pow((point.x / e1_major_radius_), 2) + pow((point.y / e1_minor_radius_), 2) - 1);
}

void RTOSafety::visualizeEllipses()
{
	e1_viz_msg_.header.stamp = e2_viz_msg_.header.stamp = this->now();

	// Color the ellipses green
	e1_viz_msg_.color.r = 0.0;
	e1_viz_msg_.color.g = 1.0;
	e1_viz_msg_.color.b = 0.0;

	e2_viz_msg_.color.r = 0.0;
	e2_viz_msg_.color.g = 1.0;
	e2_viz_msg_.color.b = 0.0;

	if(stop_laser_)
	{
		// Color the ellipse e1 red
		e1_viz_msg_.color.r = 1.0;
		e1_viz_msg_.color.g = 0.0;
		e1_viz_msg_.color.b = 0.0;
	}
	if(slow_laser_)
	{
		// Color the ellipse e2 red
		e2_viz_msg_.color.r = 1.0;
		e2_viz_msg_.color.g = 0.0;
		e2_viz_msg_.color.b = 0.0;
	}

	e1_viz_pub_->publish(e1_viz_msg_);
	e2_viz_pub_->publish(e2_viz_msg_);
}