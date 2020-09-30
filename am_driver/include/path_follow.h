#ifndef AM_DRIVER_PATH_FOLLOW_H
#define AM_DRIVER_PATH_FOLLOW_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <tf/Matrix3x3.h>


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
  ros::Subscriber position_subscriber_;
  ros::Publisher cmd_vel_pub_;

  std::string path_topic_;
  std::string position_topic_;

  int current_goal_no_;
  int goal_set_size_;

  double x_goal_tmp_;
  double y_goal_tmp_;

  double goal_last_x_;
  double goal_last_y_;


  nav_msgs::Path path_;

  nav_msgs::Odometry current_position_;

  double trans_velocity_max_;
  double angular_velocity_max_;

  // 2D position prepared for controller.
  double x_current_, y_current_;
  double yaw_current_;

  void PathFollow::initializeParam();

  void PathFollow::PathCallback(const nav_msgs::Path& msg);
  void PathFollow::PositionCallback(const nav_msgs::Odometry& msg);


  void PathFollow::Rotate(const double& degree);
  void PathFollow::LineFollow(const double& distance);

  double pathFollow::distance(const double& x1, const double& y1
							const double& x2, const double& y2);

  double PathFollow::rotationRelay(const double& angular_velocity);
  double PathFollow::transRelay(const double& trans_velocity);



};

} //namespace am_driver_path

#endif // AM_DRIVER_PATH_FOLLOW_H