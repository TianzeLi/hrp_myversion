

#include "path_follow.h"

namespace am_driver_path{

PathFollow::PathFollow(
	const ros::NodeHandel& nh,
	const ros::NodeHandel& nh_private):
  nh_(nh),
  nh_private_(nh_private_)
{	
	ROS_INFO("Starting PathFollow.");

	initializeParam();

	path_subscriber_ = nh_.subscirbe(path_topic_, 10, &PathFollow::PathCallback, this);
	cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 200);

}

PathFollow::~PathFollow()
{
	ROS_INFO("Destroying PathFollow.");
}

void PathFollow::initializeParam()
{


}

void PathFollow::PathCallback(const nav_msgs::Path& msg)
{


}

void PathFollow::Rotate(const double& degree)
{

}

void PathFollow::Forward(const double& distance)
{
	
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

