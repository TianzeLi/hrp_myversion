#ifndef AM_DRIVER_PATH_FOLLOW_H
#define AM_DRIVER_PATH_FOLLOW_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <tf/LinearMath/Matrix3x3.h>

#define ANGULAR_THRESHOD 0.01
#define LINEAR_THRESHOD 0.05
#define PI 3.1415926


namespace am_driver_path{

class PathFollow
{
  public:
  PathFollow(const ros::NodeHandle& nh,
			 const ros::NodeHandle& nh_private);
  virtual ~PathFollow();

  private:


  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber path_subscriber_;
  ros::Subscriber position_subscriber_;
  ros::Publisher cmd_vel_pub_;

  std::string path_topic_;
  std::string position_topic_;

  bool initial_pose_received_;

  int current_goal_no_;
  int goal_set_size_;

  double x_goal_tmp_;
  double y_goal_tmp_;

  double last_goal_x_;
  double last_goal_y_;

  double sample_time_;
  double KP_rotate_;
  double KP_trans_;
  double p_value_;

  nav_msgs::Path path_;

  nav_msgs::Odometry current_position_;

  geometry_msgs::Twist cmd_vel_stop_;

  double trans_velocity_max_;
  double angular_velocity_max_;

  // 2D position prepared for controller.
  double x_current_, y_current_;
  double yaw_current_;

  void initializeParam();

  void PathCallback(const nav_msgs::Path& msg);
  void PositionCallback(const nav_msgs::Odometry& msg);

  int gotoGoal(const double& x_goal, const double& y_goal);
  void Rotate(const double& degree);
  void LineFollow(const double& x_goal, const double& y_goal, 
  				  const double& goal_direction);

  double distance(const double& x1, const double& y1,
							const double& x2, const double& y2);

  double rotationRelay(const double& angular_velocity);
  double transRelay(const double& trans_velocity);



};

} //namespace am_driver_path

#endif // AM_DRIVER_PATH_FOLLOW_H