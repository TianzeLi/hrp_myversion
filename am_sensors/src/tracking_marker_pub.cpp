/**
 * Subscibes to the pose topics and publishes the traces in rviz,
 * in sequentially red, orange, yellow, green, cyan, blue, purple.
 * 
 * By default, the trackers are published in /map frame,
 * and pose are in nav_msgs::Odometry
 * 
 * TODO: add covariance ellipse? 
 */

#include <ros/ros.h>
#include <ros/console.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>


class Tracker
{
private:
	nav_msgs::Odometry pose_tmp_;
	std::string pub_topic_;
	float marker_scale_;

	int frequency_;
	int color_number_;
	std::string pose_topic_;

	visualization_msgs::Marker marker_;


public:
	ros::Subscriber pose_sub;
	ros::Publisher tracker_pub;

	Tracker() {}

	Tracker(ros::NodeHandle *nh, int frequency, int color_number, std::string pose_topic, int seq)
	{	
		ROS_INFO_STREAM("In a tracker now.");
		this->frequency_ = frequency;
		this->color_number_ = color_number;
		this->pose_topic_ = pose_topic;

		std::stringstream ss_tmp;
		std::string pub_topic_tmp;
		ss_tmp << "tracking_marker" << seq;
		pub_topic_tmp = ss_tmp.str();

		ROS_INFO_STREAM("Tracker no." << seq << " will subscribe to " << pose_topic_);
		ROS_INFO_STREAM("Tracker no." << seq << " will publish to " << pub_topic_tmp);
		// pose_sub = nh->subscribe(pose_topic_, 10, &Tracker::pose_callback, this);
		tracker_pub = nh->advertise<visualization_msgs::Marker>(pub_topic_tmp, 20);

		static float rgbmat[7][3] = {
			{1.0, 0.0, 0.0},
			{1.0, 0.5, 0.0},
			{1.0, 1.0, 0.0},
			{0.0, 1.0, 0.0},
			{0.0, 1.0, 1.0},
			{0.0, 0.0, 1.0},
			{0.5, 0.0, 1.0}
		};

		marker_.header.frame_id = "/map";
		marker_.header.stamp = pose_tmp_.header.stamp;
		marker_.ns = "points";
		marker_.action = visualization_msgs::Marker::ADD;
		marker_.pose.orientation.w = 1.0;

		marker_.id = seq;

		marker_.type = visualization_msgs::Marker::POINTS;
		marker_scale_ = 0.1;
		marker_.scale.x = marker_scale_;
		marker_.scale.y = marker_scale_;
		marker_.scale.z = marker_scale_;

		// Marker color is sequentially red, orange, yellow, green, cyan, blue, purple.
		marker_.color.a = 1.0;
		marker_.color.r = rgbmat[color_number_ -1][0];
		marker_.color.g = rgbmat[color_number_ -1][1];	
		marker_.color.b = rgbmat[color_number_ -1][2];

	}

	void pose_callback(const nav_msgs::Odometry &msg)
	{	
		// ROS_INFO_STREAM("In pose_callback now.");
		pose_tmp_ = msg;
		geometry_msgs::Point p;
		p.x = pose_tmp_.pose.pose.position.x;
		p.y = pose_tmp_.pose.pose.position.y;
		p.z = pose_tmp_.pose.pose.position.z;
		marker_.points.push_back(p);
	}

	visualization_msgs::Marker getMarker(){
		return this->marker_;
	}
};

int color_picker(std::string const& color)
{
	int number = 7;
	if (color == "red") 	number = 1;
	if (color == "orange") 	number = 2;
	if (color == "yellow") 	number = 3;
	if (color == "green") 	number = 4;
	if (color == "cyan") 	number = 5;
	if (color == "blue") 	number = 6;
	if (color == "purple") 	number = 7;

	return number;
}



int main(int argc, char** argv)
{
	// Number of traces to plot.
	int count_max = 7;
	int count = 0;
	int frequency = 30;
	int i;
	int seq;
	int color_number = 0;

	std::stringstream ss_tmp;	
	std::string pose_topic_tmp;
	std::string color_tmp;
	std::string pose_topic;
	std::string color;
	


	ros::init(argc, argv, "tracking_marker_pub");
	ros::NodeHandle nh("~");

	ros::param::get("~trace_amount", count);
	ros::param::get("~frequency", frequency);
	ros::Rate r(frequency);
	ROS_INFO_STREAM("Node initialized with count = "<< count);

	// Initialize trackers.
    Tracker* tc_arr = new Tracker[count]; 
	for (i = 0; i < count; i++)
	{	
		// ROS_INFO_STREAM("In the for loop now. i = " << i);
		seq = i + 1;
		ss_tmp << "pose_topic" << seq;
		pose_topic_tmp = ss_tmp.str();
		nh.getParam(pose_topic_tmp, pose_topic);
		ss_tmp.clear();
		ss_tmp.str(std::string());
		ss_tmp << "color" << seq ;
		color_tmp = ss_tmp.str();
		nh.getParam(color_tmp, color);
		ss_tmp.clear();
		ss_tmp.str(std::string());
		color_number = color_picker(color);

		tc_arr[i] = Tracker(&nh, frequency, color_number, pose_topic, seq);
		tc_arr[i].pose_sub = nh.subscribe(pose_topic, 10, &Tracker::pose_callback, &tc_arr[i]);
	}	

	while(ros::ok())
	{
		ros::spinOnce();
		for (i = 0; i < count; i++)
		{
			tc_arr[i].tracker_pub.publish(tc_arr[i].getMarker());
			// ROS_INFO_STREAM("Now publishing marker of "<< (i+1));
		}
		r.sleep();
	}

	// ros::spin();

	ROS_INFO_STREAM("Out of the while-loop now.");

	return 0;
}