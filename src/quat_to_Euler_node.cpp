#include "ros/ros.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"

#define _USE_MATH_DEFINES
#include <math.h>

ros::Publisher pub;

void quatCallback(const geometry_msgs::Quaternion::ConstPtr& msg)
{

	double q[4] = {msg->w, msg->x, msg->y, msg->z};

	// Mathematical manipulation
	double x1 = 2.0 * (q[0]*q[1] + q[2]*q[3]);
	double x2 = 1 - 2.0*(q[1]*q[1] + q[2]*q[2]);
	double roll = atan2(x1,x2);

	double y1 = 2.0 * (q[0]*q[2] - q[3]*q[1]);
	double pitch = 0;
	if (fabs(y1) >= 1)
		pitch = copysign(M_PI/2, y1); //use 90degrees if out of range
	else
		pitch = asin(y1);

	double z1 = 2.0 * (q[0]*q[3] + q[1]*q[2]);
	double z2 = 1 - 2.0*(q[2]*q[2] + q[3]*q[3]);
	double yaw = atan2(z1,z2);

	geometry_msgs::Vector3 pub_msg;
	pub_msg.x = roll;
	pub_msg.y = pitch;
	pub_msg.z = yaw;
	pub.publish(pub_msg);
	
}


int main(int argc, char **argv){
	ros::init(argc, argv, "quat_to_Euler_node");
	ros::NodeHandle nh;
	
	// TODO: Write subscribers
	ROS_INFO("\nWrite your subscriber here!\n");

	ros::Subscriber sub=nh.subscribe("quaternion",10,quatCallback);

	pub = nh.advertise<geometry_msgs::Vector3>("eulerAngle",10);
	ros::spin();
	return 0;	
}
