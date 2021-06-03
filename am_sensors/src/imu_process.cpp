/* 	
 * IMU original data processor.
 * The CPT filter by default publish in NWU, so here we rotate it into 
 * ENU and attach a proper covariance matrix. 
 *
 * For the convenience of testing and tuning, the node can optionally 
 * publihsh the tf frame named "imu_<left/right>_enu" linked to /map
 * to visualize the pose of the IMU. It depends on the relatively 
 * pose of IMU and the base_link.
 *
 */

#include <std_msgs/String.h>
#include <sstream>
#include "imu_process.h"


namespace am_sensors_imu{

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
	cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 10, &IMUProcess::CmdVelCallback, this);
	if(publish_rpy_)
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

	imu_shall_pub_ = true;
	uncertainty_coef_r_ = 1.0;
	uncertainty_coef_p_ = 1.0;
	uncertainty_coef_y_ = 1.0;

	if (!nh_private_.getParam("topic_name", topic_name_))
		topic_name_ = "imu/data";
	if (!nh_private_.getParam("frame_name", fixed_frame_))
		fixed_frame_ = "map";
	if (!nh_private_.getParam("publish_rpy", publish_rpy_))
		publish_rpy_ = false;
	if (!nh_private_.getParam("pub_test_tf", pub_test_tf_))
		pub_test_tf_ = false;	
	if (!nh_private_.getParam("publish_enu_tf", publish_enu_tf_))
		publish_enu_tf_ = false;
	if (!nh_private_.getParam("xyz", xyz))
		xyz = "0.0 0.16 0.37";
	if (!nh_private_.getParam("rpy", rpy))
		rpy = "3.142 0.0 1.571";
		std::stringstream xyz_ss(xyz);
		std::stringstream rpy_ss(rpy);
		xyz_ss >> x_ >> y_ >> z_;
		ROS_INFO("y: [%f]", y_);
		rpy_ss >> roll_ >> pitch_ >> yaw_;

	if(pub_test_tf_){
		pub_tf(x_, y_, z_, roll_, pitch_, yaw_, fixed_frame_);
		ROS_INFO("Publishing testing TF now.");
	}

	if (!nh_private_.getParam("do_covariance_adaption", do_covariance_adaption_))
 		do_covariance_adaption_ = false;
	if (!nh_private_.getParam("do_pub_control", do_pub_control_))
		do_pub_control_ = false;
	if (!nh_private_.getParam("angular_velocity_threshold", angular_velocity_threshold_))
		angular_velocity_threshold_ = 1.5;
	if (!nh_private_.getParam("trans_velocity_threshold", trans_velocity_threshold_))
		trans_velocity_threshold_ = 1.0;
 	if (!nh_private_.getParam("orientation_stddev_r", orientation_stddev_r_))
 		orientation_stddev_r_ = 0.001;
 	if (!nh_private_.getParam("orientation_stddev_p", orientation_stddev_p_))
 		orientation_stddev_r_ = 0.001;
 	if (!nh_private_.getParam("orientation_stddev_y", orientation_stddev_y_))
 		orientation_stddev_r_ = 0.001;

	ss_tmp << topic_name_ << "/enu";
	pub_topic_ = ss_tmp.str();
	orientation_dev_r_ = orientation_stddev_r_*orientation_stddev_r_;
	orientation_dev_p_ = orientation_stddev_p_*orientation_stddev_p_;
	orientation_dev_y_ = orientation_stddev_y_*orientation_stddev_y_;
}

void IMUProcess::IMUCallback(const sensor_msgs::Imu &msg)
{	
	sensor_msgs::Imu imu_tmp = msg;
	tf2::Quaternion q_nwu, q_rot, q_enu;
	std::string test_frame_id = "imu_left_enu";

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

	// Converting the orientation of IMU from NWU to ENU.
	tf2::convert(imu_tmp.orientation, q_nwu);
	geometry_msgs::Quaternion quat_msg;
	quat_msg.x=0.0;
	quat_msg.y=0.0;
	quat_msg.z=0.707;
	quat_msg.w=0.707;
   	tf2::convert(quat_msg , q_rot); 	
	q_enu = q_rot*q_nwu;
	q_enu.normalize();
	tf2::convert(q_enu, imu_tmp.orientation);

	if(imu_shall_pub_)
		imu_pub_.publish(imu_tmp);

	if(publish_rpy_)
	{
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

	if (publish_enu_tf_)
	{	
		tf2_ros::Buffer tfBuffer;
		static tf2_ros::TransformBroadcaster br;
		geometry_msgs::TransformStamped transform_stamped;
		geometry_msgs::TransformStamped transform_stamped_b2i;
		
		geometry_msgs::Vector3 vec_b2i, vec_now;
	    tf2::Quaternion q_b2i, q_base_link_enu;
		vec_b2i.x = x_;
		vec_b2i.y = y_;
		vec_b2i.z = z_;
		q_b2i.setRPY(roll_, pitch_, yaw_);

		// Transfrom that rotates from base_link to imu_link.
		transform_stamped_b2i.header.stamp = ros::Time::now();
		transform_stamped_b2i.header.frame_id = "base_link";
		transform_stamped_b2i.child_frame_id = "imu_link";
		transform_stamped_b2i.transform.rotation.x = q_b2i.x();
		transform_stamped_b2i.transform.rotation.y = q_b2i.y();
		transform_stamped_b2i.transform.rotation.z = q_b2i.z();
		transform_stamped_b2i.transform.rotation.w = q_b2i.w();

		// Transform that rotates the imu to its current enu pose.
		if (y_ < 0.0)  
			test_frame_id = "imu_right_enu";
		transform_stamped.header.stamp = msg.header.stamp;
    	transform_stamped.header.frame_id = "map";
  	   	transform_stamped.child_frame_id = test_frame_id;
		transform_stamped.transform.rotation.x = q_enu.x();
		transform_stamped.transform.rotation.y = q_enu.y();
		transform_stamped.transform.rotation.z = q_enu.z();
		transform_stamped.transform.rotation.w = q_enu.w();

		tf2::doTransform(vec_b2i, vec_now, transform_stamped_b2i);
		tf2::doTransform(vec_now, vec_now, transform_stamped);
		transform_stamped.transform.translation.x = vec_now.x;
		transform_stamped.transform.translation.y = vec_now.y;
		transform_stamped.transform.translation.z = vec_now.z;
		br.sendTransform(transform_stamped);
	}
}

void IMUProcess::CmdVelCallback(const geometry_msgs::Twist msg)
{
	double vt, vr;
	// double vr_max = angular_velocity_threshold_;
	// double vt_max = trans_velocity_threshold_;
	// Convert to the absolute value here.
	vt = std::abs(msg.linear.x);
	vr = std::abs(msg.angular.z);

	if (do_pub_control_)
	{
		if (vt > trans_velocity_threshold_ || vr > angular_velocity_threshold_)
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
		uncertainty_coef_p_ = 1.0 + 10*vr + 5*vt;
		uncertainty_coef_y_ = 1.0 + 20*vr + 10*vt;
	}
}

void IMUProcess::pub_tf(double x, double y, double z, double r, double p, double yaw, const std::string frame_name)
{	
	ros::Rate rate(10.0);
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(x_, y_, z_) ); // might need to inverse the transform
	tf::Quaternion q;
	q.setRPY(r, p, yaw); // might need to inverse the transform
	transform.setRotation(q);

	while (nh_.ok()){
		br.sendTransform(tf::StampedTransform(transform.inverse(), 
		ros::Time::now(), frame_name, "map_test")); // first parent frame name, then child frame name
		rate.sleep();
	}
}

}// namespace am_sensors_imu



int main(int argc, char** argv)
{
	ros::init(argc, argv, "IMUProcess");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	am_sensors_imu::IMUProcess imu_instance(nh, nh_private);
	ros::spin();
	return 0;
}

