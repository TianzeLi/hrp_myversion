#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include<nav_msgs/Odometry.h>
#include<geometry_msgs/Point.h>


#define MAP_FRAME_NAME 	"/map"
#define POSE_TOPIC 		"/odometry/filtered"
#define POSE_TYPE 		nav_msgs::Odometry 
POSE_TYPE pose_tmp;


void pose_callback(const POSE_TYPE &msg)
{
	pose_tmp = msg;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "tracking_marker_pub");
	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe(POSE_TOPIC, 10, &pose_callback);
	ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("tracking_marker", 20);
	ros::Rate r(20);
	ros::spinOnce();

	visualization_msgs::Marker marker;
	marker.header.frame_id = MAP_FRAME_NAME;
	marker.header.stamp = ros::Time::now();
	marker.ns = "points";
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.orientation.w = 1.0;

	marker.id = 0;

	marker.type = visualization_msgs::Marker::POINTS;
	marker.scale.x = 0.07;
	marker.scale.y = 0.07;
	marker.scale.z = 0.07;

	// Marker color is red.
	marker.color.r = 1.0;
	marker.color.a = 1.0;

	while(ros::ok())
	{
		geometry_msgs::Point p;
		p.x = pose_tmp.pose.pose.position.x;
		p.y = pose_tmp.pose.pose.position.y;
		p.z = pose_tmp.pose.pose.position.z;

		marker.points.push_back(p);

		marker_pub.publish(marker);

		ros::spinOnce();
		r.sleep();
	}
	
	return 0;
}