#include <std_msgs/String.h>
#include <sstream>

#include "imu_process.h"

namespace am_sensors {

IMUProcess::IMUProcess(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& nh_private):
  nh_(nh),
  nh_private_(nh_private)
{
  	ROS_INFO("Starting IMUProcess.");

  	initializeParams();

	imu_pub_= nh_.advertise<sensor_msgs::Imu>(pub_topic_, 200);
	imu_sub_ = nh_.subscribe(topic_name_, 10, &IMUProcess::IMUCallback, this);
	cmd_vel_sub_ = nh_.subscribe("/cmd/vel", 10, &IMUProcess::CmdVelCallback, this);
	imu_pub_rpy_ = nh_.advertise<geometry_msgs::Vector3>("rpy", 200);
}

IMUProcess::~IMUProcess()
{
	ROS_INFO("Destorying IMUProcess.");
}

void IMUProcess::initializeParams()
{	
	std::string xyz;
    std::string rpy;
    std::stringstream ss_tmp;
	double x, y, z;
	double roll, pitch, yaw;

	imu_shall_pub_ = true;
	uncertainty_coef_r_ = 1.0;
	uncertainty_coef_p_ = 1.0;
	uncertainty_coef_y_ = 1.0;

	if (!nh_private_.getParam("topic_name", topic_name_))
		topic_name_ = "imu/data";
	if (!nh_private_.getParam("fixed_frame", fixed_frame_))
		fixed_frame_ = "map";
	if (!nh_private_.getParam("pub_test_tf", pub_test_tf_))
		pub_test_tf_ = "false";
	if(pub_test_tf_){
		if (!nh_private_.getParam("xyz", xyz))
			xyz = "0.0 0.16 0.43";
		if (!nh_private_.getParam("rpy", rpy))
			rpy = "3.142 0.0 0.0";
		std::stringstream xyz_ss(xyz);
		std::stringstream rpy_ss(rpy);
		xyz_ss >> x_ >> y_ >> z_;
		rpy_ss >> roll_ >> pitch_ >> yaw_;
		pub_tf(x, y, z, roll, pitch, yaw, fixed_frame_);
		ROS_INFO_STREAM("Publishing testing TF now.");
	}

	if (!nh_private_.getParam("do_covariance_adaption", do_covariance_adaption_))
 		do_covariance_adaption_ = false;
	if (!nh_private_.getParam("do_pub_control", do_pub_control_))
		do_pub_control_ = false;
	if (do_pub_control_)
	{
		if (!nh_private_.getParam("angular_velocity_threshod", angular_velocity_threshod_))
			angular_velocity_threshod_ = 0.1;
		if (!nh_private_.getParam("trans_velocity_threshod", trans_velocity_threshod_))
			angular_velocity_threshod_ = 1.0;
	}

 	if (!nh_private_.getParam("orientation_stddev_r", orientation_stddev_r_))
 		orientation_stddev_r_ = 0.001;
 	if (!nh_private_.getParam("orientation_stddev_p", orientation_stddev_p_))
 		orientation_stddev_r_ = 0.001;
 	if (!nh_private_.getParam("orientation_stddev_y", orientation_stddev_y_))
 		orientation_stddev_r_ = 0.001;


	ss_tmp << topic_name_ << "/processed";
	pub_topic_ = ss_tmp.str();
	orientation_dev_r_ = orientation_stddev_r_*orientation_stddev_r_;
	orientation_dev_p_ = orientation_stddev_p_*orientation_stddev_p_;
	orientation_dev_y_ = orientation_stddev_y_*orientation_stddev_y_;
}

void IMUProcess::IMUCallback(const sensor_msgs::Imu &msg)
{	
	sensor_msgs::Imu imu_tmp = msg;
	tf2::Quaternion q_nwu, q_rot, q_enu;

	imu_tmp.header.frame_id = fixed_frame_;

 	// Set the covariance for rpy in orientation.
	if (do_covariance_adaption_)
	{
		imu_tmp.orientation_covariance[0] = orientation_dev_r_*uncertainty_coef_r_;
		imu_tmp.orientation_covariance[4] = orientation_dev_p_*uncertainty_coef_p_;
		imu_tmp.orientation_covariance[8] = orientation_dev_y_*uncertainty_coef_y_;
	}
	else
	{
		imu_tmp.orientation_covariance[0] = orientation_dev_r_;
		imu_tmp.orientation_covariance[4] = orientation_dev_p_;
		imu_tmp.orientation_covariance[8] = orientation_dev_y_;
	}

	// Is this part needed??
	// Converting from NWU to ENU.
	tf2::convert(imu_tmp.orientation, q_nwu);
	geometry_msgs::Quaternion quat_msg;
	quat_msg.x=0.0;
	quat_msg.y=0.0;
	quat_msg.z=0.707;
	quat_msg.w=0.707;

   	tf2::convert(quat_msg , q_rot); 
	// Problems lies here?
	q_enu = q_rot*q_nwu;  // Calculate the new orientation
	q_enu.normalize();
	// Stuff the new rotation back into the pose. This requires conversion into a msg type
	tf2::convert(q_enu, imu_tmp.orientation);

	if(imu_shall_pub_)
		imu_pub_.publish(imu_tmp);

	tf::Quaternion q(
    	msg.orientation.x,
    	msg.orientation.y,
    	msg.orientation.z,
    	msg.orientation.w);
	tf::Matrix3x3 m(q);
	geometry_msgs::Vector3 rpy;
	m.getRPY(rpy.x, rpy.y, rpy.z);
	imu_pub_rpy_.publish(rpy);
}

void IMUProcess::CmdVelCallback(const geometry_msgs::Twist msg)
{
	double vt, vr;
	vt = msg.linear.x;
	vr = msg.angular.z;
	if (do_pub_control_)
	{
		if (vt > trans_velocity_threshod_ || vr > angular_velocity_threshod_)
			imu_shall_pub_ = false;
		else 
			imu_shall_pub_ = true;
	}
	else 
			imu_shall_pub_ = true;

	if (do_covariance_adaption_)
	{	
		// customized coeffecient, should be tuned. 
		uncertainty_coef_r_ = 1.0 + 10*vr + 5*vt;
		uncertainty_coef_r_ = 1.0 + 10*vr + 5*vt;
		uncertainty_coef_r_ = 1.0 + 20*vr + 10*vt;
	}
}

void IMUProcess::pub_tf(double x, double y, double z, double r, double p, double yaw, const std::string frame_name)
{	
	ros::Rate rate(10.0);

	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(x, y, z) ); // might need to inverse the transform
	tf::Quaternion q;
	q.setRPY(r, p, yaw); // might need to inverse the transform
	transform.setRotation(q);

	while (nh_.ok()){
		br.sendTransform(tf::StampedTransform(transform.inverse(), 
		ros::Time::now(), frame_name, "map_test")); // first parent frame name, then child frame name
		rate.sleep();
	}
}

}// namespace am_sensors




int main(int argc, char** argv)
{
	ros::init(argc, argv, "IMUProcess");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	am_sensors::IMUProcess imu_instance(nh, nh_private);
	ros::spin();
	return 0;
}


  // <joint name="imu01_link_joint" type="fixed">
  //   <parent link="base_link" />
  //   <child link="imu01_link" />
  //   <origin xyz="0 0.16 0.34" rpy="3.142 0 1.571" />
  // </joint>
