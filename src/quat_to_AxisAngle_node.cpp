#include "ros/ros.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Twist.h"

ros::Publisher pub;

void quatCallback(const geometry_msgs::Quaternion::ConstPtr& msg)
{
	
	double q[4] = {msg->w, msg->x, msg->y, msg->z};

	// Mathematical manipulation
	geometry_msgs::Twist pub_msg;

	double denominator = sqrt(1-q[0]*q[0]);
	pub_msg.angular.x = 2*acos(q[0]);
	if (denominator < 0.001)
	{
		pub_msg.linear.x = q[1];
		pub_msg.linear.y = q[2];
		pub_msg.linear.z = q[3];
		
	}
	else
	{
		pub_msg.linear.x = q[1] / denominator;
		pub_msg.linear.y = q[2] / denominator;
		pub_msg.linear.z = q[3] / denominator;
	}	
	pub.publish(pub_msg);
}



int main(int argc, char **argv){
	ros::init(argc, argv, "quat_to_AxisAngle_node");
	ros::NodeHandle nh;
	
	// TODO: Write subscribers
	ROS_INFO("\nWrite your subscriber here!\n");

	ros::Subscriber sub=nh.subscribe("quaternion",10,quatCallback);
	
	pub = nh.advertise<geometry_msgs::Twist>("axisAngle",10);
	ros::spin();

	return 0;	
}
