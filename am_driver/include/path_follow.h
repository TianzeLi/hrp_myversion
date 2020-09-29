#ifndef AM_DRIVER_PATH_FOLLOW_H
#define AM_DRIVER_PATH_FOLLOW_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>


namespace am_driver_path{

class PathFollow
{
  public:
  PathFollow(const ros::NodeHandel& nh,
			 const ros::NodeHandel& nh_private);
  virtual ~PathFollow();

  private:

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber path_subscriber_;
  ros::Publisher cmd_vel_pub_;

  void PathFollow::initializeParam();

  void PathFollow::PathCallback(const nav_msgs::Path& msg);

  void PathFollow::Rotate(const double& degree);
  void PathFollow::Forward(const double& distance);


};

} //namespace am_driver_path

#endif // AM_DRIVER_PATH_FOLLOW_H