/* Subscribes to /wheel_encoder and publish the correspinding odometry. */

#include <ros/ros.h>
// #include <iomanip>
#include <am_driver/WheelEncoder.h>
// Check this
#include<geometry_msgs/TwistWithCovarianceStamped.h>
#include<nav_msgs/Odometry.h>





#define RADIANS_PER_TICK		(M_PI*2.0/1093.0)
#define WHEEL_METER_PER_TICK	(0.000704)


am_driver::WheelEncoder wheel_encoder_tmp;


// A callback function Executed each time a new pose message arrives.
void poseMessageReceived(const am_driver::WheelEncoder &msg)
{
	wheel_encoder_tmp = msg;
}

int main(int argc, char **argv)
{	

	// Initialize the ROS system and become a node.
	ros::init(argc, argv, "wheel_encoder_decoder");
	ros::NodeHandle nh;

	// Create a subscriber object.
	ros::Subscriber sub = nh.subscribe("/wheel_encoder", 
							10, &poseMessageReceived);

	// Create a publisher object
	ros::Publisher pub = nh.advertise<nav_msgs::Odometry>
							("/wheel_encoder_decoded", 1000);


	// Running at 10Hz
	ros::Rate loop_rate(10);
	ros::spinOnce();


	while(ros::ok())
	{
		// Create and fill in the message
		nav_msgs::Odometry odom;
		geometry_msgs::TwistWithCovarianceStamped tcs;
		
				
		// This represents an estimate of a position and velocity in free space.  
		// The pose in this message should be specified in the coordinate frame given by header.frame_id.
		// The twist in this message should be specified in the coordinate frame given by the child_frame_id
		odom.header.frame_id = "odom";
		odom.header.stamp = wheel_encoder_tmp.header.stamp;
		odom.child_frame_id = "base_link";
		// Angular speed of each wheel.
		// msg.lwheel;
		// msg.rwheel;


		tcs.twist.twist.linear.x = 
			(wheel_encoder_tmp.lwheel + wheel_encoder_tmp.rwheel)*10.0/2.5;
		tcs.twist.twist.linear.y = 0;

		// may need to fix
		tcs.twist.twist.angular.z = 
			(-wheel_encoder_tmp.lwheel + wheel_encoder_tmp.rwheel)*20.0/2.5/0.4645;

		// tcs.twist.covariance = 

		odom.twist = tcs.twist;

	    odom.twist.covariance[0] = 0.001;
	    odom.twist.covariance[7] = 0.001;
	    // odom.twist.covariance[14] = 1000.0;
	    // odom.twist.covariance[21] = 1000.0;
	    // odom.twist.covariance[28] = 1000.0;
	    odom.twist.covariance[35] = 0.001;

		// Publish the message.
		pub.publish(odom);

		// // Send a message to rosout with the details
		// ROS_INFO_STREAM("Sending random velocity command: "
		// << " linear=" << wheel_encoder_tmp.linear.x
		// << " angular=" << wheel_encoder_tmp.angular.z);

		ros::spinOnce();
		loop_rate.sleep();

	}

	// ROS_INFO_STREAM(std::setprecision(2) << std::fixed 
	// 	<< "position=(" << msg.x << ", " << msg.y << ")"
	// 	<< " direction=" << msg.theta); 


	return 0;
}




/*
		std_msgs/Header header
		  uint32 seq
		  time stamp
		  string frame_id
		string child_frame_id
		geometry_msgs/PoseWithCovariance pose
		  geometry_msgs/Pose pose
		    geometry_msgs/Point position
		      float64 x
		      float64 y
		      float64 z
		    geometry_msgs/Quaternion orientation
		      float64 x
		      float64 y
		      float64 z
		      float64 w
		  float64[36] covariance
		geometry_msgs/TwistWithCovariance twist
		  geometry_msgs/Twist twist
		    geometry_msgs/Vector3 linear
		      float64 x
		      float64 y
		      float64 z
		    geometry_msgs/Vector3 angular
		      float64 x
		      float64 y
		      float64 z
		  float64[36] covariance
		*/
		/*
		 std_msgs/Header header
		   uint32 seq
		   time stamp
		   string frame_id
		 geometry_msgs/TwistWithCovariance twist
		   geometry_msgs/Twist twist
		     geometry_msgs/Vector3 linear
		       float64 x
		       float64 y
		       float64 z
		     geometry_msgs/Vector3 angular
		       float64 x
		       float64 y
		       float64 z
		   float64[36] covariance
		*/

