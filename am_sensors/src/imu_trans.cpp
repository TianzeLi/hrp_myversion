#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <sstream>
#include "geometry_msgs/Vector3.h"
#include <tf/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


class imu_sub_pub
{
public:
	
	imu_sub_pub()
	{
		ros::param::get("~frame_name", frame_name);
		ros::param::get("~topic_name", topic_name);
		ros::param::get("~pub_testtf", pub_testtf);

		if(pub_testtf){
			ros::param::get("~xyz", xyz);
			ros::param::get("~rpy", rpy);
			std::stringstream xyz_ss(xyz);
			std::stringstream rpy_ss(rpy);

			double x, y, z, roll, pitch, yaw;
			xyz_ss >> x >> y >> z;
			rpy_ss >> roll >> pitch >> yaw;
			pub_tf(x, y, z, roll, pitch, yaw, frame_name);
			ROS_INFO_STREAM("Publishing testing TF now.");
		}

		else{
			ROS_INFO_STREAM("NOT publishing testing TF.");
		}

		std::stringstream ss_tmp;
		ss_tmp << topic_name << "/processed";
		imu_new_topic = ss_tmp.str();

		imu_pub_= nh_.advertise<sensor_msgs::Imu>(imu_new_topic, 200);
		imu_sub_ = nh_.subscribe(topic_name, 10, &imu_sub_pub::imu_callback, this);
		imu_pub_rpy_ = nh_.advertise<geometry_msgs::Vector3>("rpy", 200);
	}

	void imu_callback(const sensor_msgs::Imu &msg)
	{	
		sensor_msgs::Imu imu_tmp = msg;
		tf2::Quaternion q_nwu, q_rot, q_enu;

		// Set the covariance for rpy in orientation.
		imu_tmp.header.frame_id = frame_name;
		imu_tmp.orientation_covariance[0] = 0.05;
		imu_tmp.orientation_covariance[4] = 0.05;
		imu_tmp.orientation_covariance[8] = 0.1;

		// Converting from NWU to ENU.
		tf2::convert(imu_tmp.orientation, q_nwu);
		double r = 0.0, p = 0.0, y = +1.57080;  // Rotate the previous pose by 180* about X
		q_rot.setRPY(r, p, y);
		// Problems lies here?
		q_enu = q_nwu*q_rot;  // Calculate the new orientation
		q_enu.normalize();
		// Stuff the new rotation back into the pose. This requires conversion into a msg type
		tf2::convert(q_enu, imu_tmp.orientation);

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

	void pub_tf(double x, double y, double z, double r, double p, double yaw, const std::string frame_name)
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


private:
  ros::NodeHandle nh_; 
  ros::Publisher imu_pub_;
  ros::Subscriber imu_sub_;
  ros::Publisher imu_pub_rpy_;

  std::string topic_name;
  std::string imu_new_topic;
  std::string frame_name;
  std::string xyz;
  std::string rpy;
  int pub_testtf;

};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "imu_transform_pub");
	imu_sub_pub imu_object;

	ros::spin();
	return 0;
}


  // <joint name="imu01_link_joint" type="fixed">
  //   <parent link="base_link" />
  //   <child link="imu01_link" />
  //   <origin xyz="0 0.16 0.34" rpy="3.142 0 1.571" />
  // </joint>
