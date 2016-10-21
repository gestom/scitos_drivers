#include <ros/console.h>
#include <stdlib.h>
#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

//odometry correction values
double corrX,corrY;
//last measurements
double lastX,lastY,lastT;
//simple statistical model
long int n;
double s,ss;

ros::Publisher odomPub;
//tf::TransformBroadcaster tfPub;

void odomCallback(const nav_msgs::Odometry &msg)
{
	float x = msg.pose.pose.position.x-corrX;
	float y = msg.pose.pose.position.y-corrY;
	if (lastX != 0 || lastY != 0){
		double d = sqrt((x-lastX)*(x-lastX)+(y-lastY)*(y-lastY));
		double t = msg.header.stamp.toSec()-lastT;
		double error = d-fabs(msg.twist.twist.linear.x*t);
		//if the robot actually moves
		if (d>0){
			//and the model has enough data
			if (n>100){
				double mean = s/n; 
				double std = ss/n-s*s/n/n; 
				//printf("%.5f %.5f %.5f %.5f %.5f\n",d,msg.twist.twist.linear.x,t,error,(error-mean)/sqrt(std));
				//detect outliers that are less than 10 sigma probable
				if (std != 0 && fabs(error-mean)/sqrt(std) > 10)
				{
					ROS_WARN("Odometry glitch detected. Position jump from %.3f %.3f to %.3f %.3f. Probability %.0f sigma.",lastX,lastY,x,y,fabs(error-mean)/sqrt(std));
					//this is clearly an odometry glitch, so update corrective constants
					corrX = (x-lastX);	
					corrY = (y-lastY);
					//correct the odometry and model error
					x = msg.pose.pose.position.x-corrX;
					y = msg.pose.pose.position.y-corrY;
					error = 0;
				}
			}
			//update the error model
			s = s + error;
			ss = ss + error*error;
			n++;
		}
	}
	nav_msgs::Odometry outMsg = msg;
	outMsg.pose.pose.position.x = x;
	outMsg.pose.pose.position.y = y;
	lastX = x; 
	lastY = y;
	lastT = msg.header.stamp.toSec();

	odomPub.publish(outMsg);

	geometry_msgs::TransformStamped tf;
	tf.header.stamp = msg.header.stamp;
	tf.header.frame_id = "/odom";
	tf.child_frame_id = "/base_footprint";

	geometry_msgs::TransformStamped odom_tf;
	tf.transform.translation.x = msg.pose.pose.position.x-corrX;
	tf.transform.translation.z = 0.0;
	tf.transform.rotation = msg.pose.pose.orientation;
	//tfPub.sendTransform(tf);
}

int main(int argc,char* argv[])
{
	lastX=lastY=corrX=corrY=s=ss=n=0;
	ros::init(argc, argv, "odom_correction");
	ros::NodeHandle *nh = new ros::NodeHandle(); 

	ros::Subscriber odomSub = nh->subscribe("odom", 1, odomCallback);
	odomPub = nh->advertise<nav_msgs::Odometry>("/odom_corr", 1);

	ros::spin();

	return 0;
}
