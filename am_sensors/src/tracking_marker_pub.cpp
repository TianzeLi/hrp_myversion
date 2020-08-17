/**
 * Subscibes to the pose topics and publishes the trace in rviz,
 * in sequentially red, orange, yellow, green, cyan, blue, purple.
 * 
 * By default, the trackers are published in /map frame,
 * and pose are in nav_msgs::Odometry
 * 
 * TODO: add covariance ellipse? 
 */

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>


class tracker
{
public:

	tracker(std::string pose_topic, std::string color, int frequency)
	{
		ros::Subscriber sub = nh_.subscribe(pose_topic, 10, &tracker::pose_callback, this);
		ros::Publisher marker_pub = nh_.advertise<visualization_msgs::Marker>("tracking_marker", 20);
		ros::Rate r(frequency);

		visualization_msgs::Marker marker;
		float* color_rgb[3];

		marker.header.frame_id = "/map";
		marker.header.stamp = pose_tmp_.header.stamp;
		marker.ns = "points";
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.orientation.w = 1.0;

		marker.id = 0;

		marker.type = visualization_msgs::Marker::POINTS;
		marker_scale_ = 0.1;
		marker.scale.x = marker_scale_;
		marker.scale.y = marker_scale_;
		marker.scale.z = marker_scale_;

		// Marker color is sequentially red, orange, yellow, green, cyan, blue, purple.
		marker.color.a = 1.0;
		color_rgb = tracker::color_picker(color);
		marker.color.r = color_rgb[0];
		marker.color.g = color_rgb[1];	
		marker.color.b = color_rgb[2];

		while(ros::ok())
		{
			geometry_msgs::Point p;
			p.x = pose_tmp_.pose.pose.position.x;
			p.y = pose_tmp_.pose.pose.position.y;
			p.z = pose_tmp_.pose.pose.position.z;
			marker.points.push_back(p);
			marker_pub.publish(marker);
			r.sleep();
		}
	}

	void pose_callback(const nav_msgs::Odometry &msg)
	{
		pose_tmp_ = msg;
	}

	float* color_picker(std::string const& color)
	{
		static float rgbmat[7][3] = {
			{1.0, 0.0, 0.0},
			{1.0, 0.5, 0.0},
			{1.0, 1.0, 0.0},
			{0.0, 1.0, 0.0},
			{0.0, 1.0, 1.0},
			{0.0, 0.0, 1.0},
			{0.5, 0.0, 1.0}
		};

		int i = 7;
		if (color == "red") 	i = 1;
		if (color == "orange") 	i = 2;
		if (color == "yellow") 	i = 3;
		if (color == "green") 	i = 4;
		if (color == "cyan") 	i = 5;
		if (color == "blue") 	i = 6;
		if (color == "purple") 	i = 7;

		return rgbmat[i-1];
	}

private:
	ros::NodeHandle nh_;
	ros::Subscriber pose_sub;
	ros::Publisher tracker_pub;

	std::string pose_topic;
	nav_msgs::Odometry pose_tmp_;
	float marker_scale_;
};



int main(int argc, char** argv)
{
	// Number of traces to plot.
	int count_max = 7;
	int count = 0;
	int frequency = 30;
	std::stringstream ss_tmp;
	std::string pose_topic_tmp;
	std::string pose_topic;
	std::string color_tmp;
	std::string color;
	int i;

	ros::init(argc, argv, "tracking_marker_pub");

	ros::param::get("trace_amount", count);
	ros::param::get("frequency", frequency);


	// count trackerd topic numbers
	for (i = 0; i < count; i++)
	{	
		ss_tmp << "pose_topic" << i;
		pose_topic_tmp = ss_tmp.str();
		ss_tmp << "color" << i ;
		color_tmp = ss_tmp.str();
		ros::param::get(pose_topic_tmp, pose_topic);float
		ros::param::get(color_tmp, color);
		// Necessary to change to different names for different instances?
		tracker tracker(pose_topic, color, frequency);
	}	
	
	ros::spin();
	return 0;
}