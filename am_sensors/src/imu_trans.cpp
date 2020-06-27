#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <sstream>
#include "geometry_msgs/Vector3.h"


class imu_sub_pub
{
public:
	
	imu_sub_pub()
	{
		ros::param::get("~frame_name",  frame_name);
		ros::param::get("~topic_name", topic_name);
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
		imu_tmp.header.frame_id = frame_name;
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

private:
  ros::NodeHandle nh_; 
  ros::Publisher imu_pub_;
  ros::Subscriber imu_sub_;
  ros::Publisher imu_pub_rpy_;

  std::string topic_name = "notgiven";
  std::string imu_new_topic = "notgiven";
  std::string frame_name = "notgiven";

};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "imu_transform_pub");
	imu_sub_pub imu_object;

	ros::spin();
	return 0;
}