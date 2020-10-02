/* 	
 * Path following controller, which consists of:
 * 1. A controller for rotation and line following.  
 * 2. A policy to conduct rotation and transfer sequentially and renew goal.
 *
 */

#include <math.h>
#include <tf/transform_datatypes.h>
// #include <xmlrpc/XmlRpcDecl.h>
#include <XmlRpcException.h>

#include "path_follow.h"

namespace am_driver_path{

PathFollow::PathFollow(
	const ros::NodeHandle& nh,
	const ros::NodeHandle& nh_private):
  nh_(nh),
  nh_private_(nh_private_)
{	
	ROS_INFO("Starting PathFollow node.");

	initializeParam();

	// path_subscriber_ = nh_.subscirbe(path_topic_, 10, &PathFollow::PathCallback, this);
	position_subscriber_ = nh_.subscribe(position_topic_, 10, &PathFollow::PositionCallback, this);
	cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 200);

	// Try to navigate to the goals one by one, untill reach all. 
	while (current_goal_no_ < goal_set_size_ )
	{	
		x_goal_tmp_ = path_.poses[current_goal_no_].pose.position.x;
		y_goal_tmp_ = path_.poses[current_goal_no_].pose.position.y;

		if (initial_pose_received_)
		{
			if (gotoGoal(x_goal_tmp_, y_goal_tmp_))
			{
				ROS_INFO_STREAM("Goal no." << (current_goal_no_ + 1) << "["
								<< x_goal_tmp_ << ", " << y_goal_tmp_ 
								<< "] reached" << std::endl);
			}
			// Here use the current location as the last goal's.  
			last_goal_x_ = x_current_;
			last_goal_y_ = y_current_;
			current_goal_no_++;
		}
		ros::spinOnce();
	}

	ROS_INFO("All goal reached!");
}

PathFollow::~PathFollow()
{
	ROS_INFO("Destroying PathFollow.");
}

void PathFollow::initializeParam()
{	
	double x_tmp, y_tmp;
	geometry_msgs::PoseStamped pose_stamped_tmp;

	initial_pose_received_ = false;

	current_goal_no_ = -1;
	goal_set_size_ = -1;

	x_goal_tmp_ = 0.0;
    y_goal_tmp_ = 0.0;

	last_goal_x_ = 0.0;
	last_goal_y_ = 0.0;

	cmd_vel_stop_.linear.x = 0.0;
	cmd_vel_stop_.angular.z = 0.0;

	path_.header.stamp = ros::Time::now();

	XmlRpc::XmlRpcValue path_list;

	if (!nh_private_.getParam("path_frame", path_.header.frame_id))
		path_.header.frame_id = "map";

	if (!nh_private_.getParam("path_planned", path_list))
		ROS_INFO("Queue Empty.");
	else 
	{
		ROS_INFO("Fetching queue.");

		goal_set_size_ = path_list.size()/2;

		ROS_INFO_STREAM("goal_set_size_ = " << goal_set_size_<< std::endl);

		ROS_ASSERT(path_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

		for (int i = 0; i < path_list.size(); i = i + 2) 
		{
		  	ROS_ASSERT(path_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);	  

		  	x_tmp = static_cast<double>(path_list[i]);
		  	y_tmp = static_cast<double>(path_list[i+1]);
		  	ROS_INFO_STREAM("Goal no." << (i/2 + 1) << " is set for [" << x_tmp
		  					<< ", " << y_tmp << "]" << std::endl);

		    pose_stamped_tmp.pose.position.x = x_tmp;
		    pose_stamped_tmp.pose.position.y = y_tmp;
		    pose_stamped_tmp.header.seq = i/2 + 1;
		    pose_stamped_tmp.header.stamp = path_.header.stamp;
		    pose_stamped_tmp.header.frame_id = path_.header.frame_id;
		    path_.poses.push_back(pose_stamped_tmp); 
		}
	}

	if (!nh_private_.getParam("position_topic", position_topic_))
	{
		ROS_INFO("position_topic not specified."); 
		position_topic_ = "/odom";
	}

	if (!nh_private_.getParam("sample_time", sample_time_))
		sample_time_ = 0.05;

	if (!nh_private_.getParam("KP_rotate", KP_rotate_))
		KP_rotate_ = 0.1;
	if (!nh_private_.getParam("KP_trans", KP_trans_))
		KP_trans_ = 0.1;
	if (!nh_private_.getParam("p_value", p_value_))
		p_value_ = 1.0;	

	if (!nh_private_.getParam("trans_velocity_max", trans_velocity_max_))
		trans_velocity_max_ = 0.5;

	if (!nh_private_.getParam("angular_velocity_max", angular_velocity_max_))
		angular_velocity_max_ = 1.0;

	// Set the first point as the current goal.
	current_goal_no_ = 0;

	ROS_INFO("Now initialized!");

}

// void PathFollow::PathCallback(const nav_msgs::Path& msg)
// {
// 	// Update the queue there is any new.

// }

void PathFollow::PositionCallback(const nav_msgs::Odometry& msg)
{	
	current_position_ = msg;
	tf::Quaternion quat_tf;

	// Extract the x and y element of the x axix.
	// Matrix3x3 (const Quaternion &q)
	tf::quaternionMsgToTF(msg.pose.pose.orientation , quat_tf);
	tf::Matrix3x3 mat_tmp(quat_tf);
	// atan(y,x), in radians, return [-pi, pi]
	yaw_current_ = std::atan2(mat_tmp[1][0], mat_tmp[0][0]);
	x_current_ = msg.pose.pose.position.x;
	y_current_ = msg.pose.pose.position.y;

	initial_pose_received_ = true;
	ROS_DEBUG("Finished one position update.");

}

int PathFollow::gotoGoal(const double& x_goal, const double& y_goal)
{	
	double goal_direction = 0;
	
	ROS_INFO_STREAM("Now start to transport to [" << x_goal << ", "
					<< y_goal << "]" << std::endl);

	// return [-pi, pi]
	goal_direction = std::atan2((y_goal - y_current_), (x_goal - x_current_));

	// First rotate.
	Rotate(goal_direction);
	ROS_INFO("Rotation done");
	// Then move forward.
	LineFollow(x_goal, y_goal, goal_direction);
	ROS_INFO("Line follow done.");

	// If sucssful.
	return 0;
}

// Controlling the rotation velocity, change the robot’s 
// orientation θ such that it is close to a desired angle;
void PathFollow::Rotate(const double& goal_direction)
{	
	double angle_difference = 0.0;
	double angular_velocity_input = 0.0;
	geometry_msgs::Twist vel_cmd;

	while(std::abs(yaw_current_ - goal_direction) > ANGULAR_THRESHOD)
	{	
		vel_cmd.linear.x = 0.0;
		angle_difference = goal_direction - yaw_current_;


		if (angle_difference > PI)
			angle_difference -= 2*PI;
		if (angle_difference < -PI)
			angle_difference += 2*PI;

		angular_velocity_input = KP_rotate_*angle_difference; 
		vel_cmd.angular.z = rotationRelay(angular_velocity_input);
		cmd_vel_pub_.publish(vel_cmd);
		ros::Duration(sample_time_).sleep();
		ros::spinOnce();
	}
	cmd_vel_pub_.publish(cmd_vel_stop_);
}

void PathFollow::LineFollow(const double& x_goal, const double& y_goal, const double& goal_direction)
{
	double d_p = 0.0;
	double angular_velocity_input = 0.0;
	double trans_velocity_input = 0.0;
	geometry_msgs::Twist vel_cmd;

	while(distance(x_current_, y_current_, x_goal, y_goal)> LINEAR_THRESHOD)
	{	
		ROS_DEBUG_STREAM("Current position is [" << x_current_ << ", "
						<< y_current_ << "] " << std::endl);	
		trans_velocity_input = KP_trans_*distance(x_current_, y_current_, x_goal, y_goal); 
		ROS_DEBUG_STREAM("Current distance = " << trans_velocity_input/KP_trans_ <<std::endl);
		vel_cmd.linear.x = transRelay(trans_velocity_input);

		d_p = (x_current_- last_goal_x_)*sin(goal_direction) 
		      - (y_current_ - last_goal_y_)*cos(goal_direction)
			  + p_value_*sin(goal_direction - yaw_current_);
		angular_velocity_input = KP_rotate_*d_p; 
		vel_cmd.angular.z = rotationRelay(angular_velocity_input);

		cmd_vel_pub_.publish(vel_cmd);
		ros::Duration(sample_time_).sleep();
		ros::spinOnce();
	}
	cmd_vel_pub_.publish(cmd_vel_stop_);
}

double PathFollow::distance(const double& x1, const double& y1,
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
	ros::NodeHandle nh;
	ros::NodeHandle nh_private;
	am_driver_path::PathFollow path_follower(nh, nh_private);
	ros::spin();
	return 0;
}

