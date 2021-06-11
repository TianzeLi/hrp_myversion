#ifndef AM_SENSORS_IMU_PROCESS_H
#define AM_SENSORS_IMU_PROCESS_H

#include <ros/ros.h>
#include <ros/console.h>
#include <tf/tf.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace am_sensors_imu{

class IMUProcess 
{
  public:	
	IMUProcess(const ros::NodeHandle& nh,
             const ros::NodeHandle& nh_private);
	virtual ~IMUProcess();

  private:

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Publisher imu_pub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber cmd_vel_sub_; 
  ros::Publisher imu_pub_rpy_;
  // static tf2_ros::TransformBroadcaster tf_broadcaster_;

  std::string topic_name_;
  std::string pub_topic_;
  std::string fixed_frame_;
  bool align_with_clock_;
  bool imu_shall_pub_;
  bool publish_rpy_;
  bool pub_test_tf_;
  bool publish_enu_tf_;
  double x_, y_, z_;
  double roll_, pitch_, yaw_;

  double orientation_stddev_r_;
  double orientation_stddev_p_;
  double orientation_stddev_y_;
  double orientation_dev_r_;
  double orientation_dev_p_;
  double orientation_dev_y_;
  double uncertainty_coef_r_;
  double uncertainty_coef_p_;
  double uncertainty_coef_y_;

  bool do_covariance_adaption_;
  bool do_pub_control_;
  double trans_velocity_threshold_;
  double angular_velocity_threshold_;

  void initializeParams();
  void IMUCallback(const sensor_msgs::Imu &msg);
  void CmdVelCallback(const geometry_msgs::Twist msg);
  void pub_tf(double x, double y, double z, double r, double p, double yaw, const std::string frame_name);
};

} // namespace am_sensors_imu

#endif // AM_SENSORS_IMU_PROCESS_H

