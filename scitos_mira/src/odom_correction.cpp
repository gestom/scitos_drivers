#include <stdlib.h>
#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

float lastX,lastY,corrX,corrY;

void odomCallback(const nav_msgs::Odometry &msg)
{	
/*	odometry_pub_.publish(msg);

	geometry_msgs::TransformStamped tf;
	tf.header.stamp = odom_time;
	tf.header.frame_id = "/odom";
	tf.child_frame_id = "/base_footprint";

	geometry_msgs::TransformStamped odom_tf;
	tf.transform.translation.x = data->value().pose.x()-corrX;
	tf.transform.translation.y = data->value().pose.y()-corrY;
	tf.transform.translation.z = 0.0;
	tf.transform.rotation = msg->pose.orientation;
	tf_broadcaster.sendTransform(tf);*/
}

int main(int argc,char* argv[])
{
	/*lastX=lastY=corrX=corrY=0;
	ros::init(argc, argv, "odom_correction");
	nh = new ros::NodeHandle;

	ros::Subscriber subodo = nh->subscribe("odom_raw", 1, odomCallback);

	ros::spin();*/

	return 0;
}
