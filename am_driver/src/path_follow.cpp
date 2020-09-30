/* 	
 * Path following controller, which consists of:
 * 1. A controller for rotation and line following.  
 * 2. A state machine to conduct rotation and transfer sequentially and renew goal.
 *
 */

#include <math.h>

#include "path_follow.h"

namespace am_driver_path{

PathFollow::PathFollow(
	const ros::NodeHandel& nh,
	const ros::NodeHandel& nh_private):
  nh_(nh),
  nh_private_(nh_private_)
{	
	ROS_INFO("Starting PathFollow node.");

	initializeParam();

	// path_subscriber_ = nh_.subscirbe(path_topic_, 10, &PathFollow::PathCallback, this);
	position_subscriber_ = nh_.subscirbe(position_topic_, 10, &PathFollow::PositionCallback, this);
	cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 200);



	//
	while (current_goal_no_ <= goal_set_size_ )
	{	
		x_goal_tmp_ = path.poses[current_goal_no_].position.x;
		y_goal_tmp_ = path.poses[current_goal_no_].position.y;

		gotoGoal(x_goal_tmp_, y_goal_tmp_);

		goal_last_x_ = x_current_;
		goal_last_y_ = y_current_;
		current_goal_no_++;
	}
}

PathFollow::~PathFollow()
{
	ROS_INFO("Destroying PathFollow.");
}

void PathFollow::initializeParam()
{	
	current_goal_no_ = -1;
	goal_set_size_ = -1;

	x_goal_tmp_ = 0.0;
    y_goal_tmp_ = 0.0;

	goal_last_x_ = 0.0;
	goal_last_y_ = 0.0;


	path_.header.stamp = ros::Time::now();

	if (!nh_private_.getParam("path_planned", path_.poses))
		ROS_INFO("Queue Empty."); 

	if (!nh_private_.getParam("position_topic", position_topic_))
		ROS_INFO("position_topic not specified."); 


	if (!nh_private_.getParam("path_frame", path_.header.frame_id))
		path_.header.frame_id = "map";

	goal_set_size_ = path.poses.size();
}

// void PathFollow::PathCallback(const nav_msgs::Path& msg)
// {
// 	// Update the queue there is any new.

// }

void PathFollow::PositionCallback(const nav_msgs::Odometry& msg)
{	
	current_position_ = msg;

	// Extract the x and y element of the x axix.
	// Matrix3x3 (const Quaternion &q)
	tf::Matrix3x3 mat_tmp(msg.pose.pose.orientation);
	// atan(y,x), in radians.
	yaw_current_ = std::atan2(mat_tmp[1][0], mat_tmp[0][0]);
	x_current_ = msg.pose.pose.position.x;
	y_current_ = msg.pose.pose.position.y;

}

int PathFollow::gotoGoal(const double& x_goal, const double& y_goal)
{	
	// Set goal.

	// First rotate.
	Rotate(degree);

	// Then move forward.
	LineFollow(x_goal, y_goal);

	// If sucssful.
	return 1;
}

// Controlling the rotation velocity, change the robot’s 
// orientation θ such that it is close to a desired angle;
void PathFollow::Rotate(const double& degree)
{	
	geometry_msgs::Twist vel_cmd;

	double KP = 0.1;

	yaw_initial = yaw_current_;
	yaw_final = yaw_initial + degree;

	while(std::abs(yaw_current_ - yaw_final) > threshod)
	{	
		vel_cmd.linear.x = 0;
		angular_velocity_pid = KP*(yaw_final - yaw_current); 
		vel_cmd.angular.z = rotationRelay(angular_velocity_pid);
		cmd_vel_pub_.pub(vel_cmd);
		ros::Duration(0.01).sleep();
	}
	cmd_vel_pub_.pub(CMD_VEL_STOP);
}

void PathFollow::LineFollow(const double& x_goal, const double& y_goal)
{
	geometry_msgs::Twist vel_cmd;

	double KP_trnas = 1.0;
	double KP_rotatinal = 1.0;

	double p = ;

	while(distance(x_current_, y_current_, x_goal, y_goal)> trans_threshod)
	{	
		
		trans_velocity_pid = KP*distance(x_current_, y_current_, x_goal, y_goal); 
		vel_cmd.linear.x = transRelay(trans_velocity_pid);

		d_p = (x_current_-x0)*sin(theta_goal) - (y_current_ - y0)*cos(theta_goal)
			  - p*sin(theta_goal + yaw_current_);
		angular_velocity_pid = KP_rotatinal*d_p; 
		vel_cmd.angular.z = rotationRelay(angular_velocity_pid);

		cmd_vel_pub_.pub(vel_cmd);
		ros::Duration(0.01).sleep();
	}
	cmd_vel_pub_.pub(CMD_VEL_STOP);
}

double pathFollow::distance(const double& x1, const double& y1
							const double& x2, const double& y2)
{
	return std::sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
}

double PathFollow::rotationRelay(const double& angular_velocity)
{
	if(std::abs(angular_velocity) < angular_velocity_max_)
		return angular_velocity;
	else if(angular_velocity > angular_velocity_max_)
		return angular_velocity_max_;
	else 
		return -angular_velocity_max_;
}

double PathFollow::transRelay(const double& trans_velocity)
{
	if(std::abs(trans_velocity) < trans_velocity_max_)
		return trans_velocity;
	else if(trans_velocity > trans_velocity_max_)
		return trans_velocity_max_;
	else 
		return -trans_velocity_max_;
}


} //namespace am_driver_path



int main(int argc, char** argv)
{
	ros::init(argc, argv, "PathFollow");
	ros::NodeHandel nh;
	ros::NodeHandel nh_private;
	am_driver_path::PathFollow path_follower();
	ros::spin();
	return 0;
}

