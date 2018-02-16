#include "ros/ros.h"
#include "geometry_msgs/Quaternion.h"

#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

ros::Publisher pub;

void matrixCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	double x,y,z,w;
	int matrixSize = msg->layout.dim[0].stride;
	double r[matrixSize];

	for (int a = 0; a < matrixSize; a = a + 1)
	{
		r[a] = msg->data[a];
		//ROS_INFO("%f",r[a]);
	}
	
	// Mathematical manipulation
	geometry_msgs::Quaternion pub_msg;
	
	double trace = r[0]+r[4]+r[8];
	
	if (trace > 0)
	{
		double denominator = sqrt(trace+1)*2; // denominator = 4 * qw
		x = (r[7]-r[5])/denominator;
		y = (r[2]-r[6])/denominator;
		z = (r[3]-r[1])/denominator;
		w = 0.25*denominator;
	} 
	else if ( (r[0] > r[4]) & (r[0] > r[8]) )
	{
		double denominator = sqrt(1+r[0]-r[4]-r[8])*2; // denominator = 4 * qx
		x = 0.25*denominator;
		y = (r[1]+r[3])/denominator;
		z = (r[2]+r[6])/denominator;
		w = (r[7]-r[5])/denominator;		
	}
	else if (r[4] > r[8])
	{
		double denominator = sqrt(1+r[4]-r[0]-r[8])*2; // denominator = 4 * qy
		x = (r[1]+r[3])/denominator;
		y = 0.25*denominator;
		z = (r[7]+r[5])/denominator;
		w = (r[2]-r[6])/denominator;		
	}
	else
	{
		double denominator = sqrt(1+r[8]-r[0]-r[4])*2; // denominator = 4 * qz
		x = (r[2]+r[6])/denominator;
		y = (r[7]+r[5])/denominator;
		z = 0.25*denominator;
		w = (r[3]-r[1])/denominator;		
	}		

	pub_msg.x = x;
	pub_msg.y = y;
	pub_msg.z = z;
	pub_msg.w = w;
	pub.publish(pub_msg);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "rotationMatrix_to_quat_node");
	ros::NodeHandle nh;
	
	// TODO: Write subscribers
	ROS_INFO("\nWrite your subscriber here!\n");

	ros::Subscriber sub=nh.subscribe("rotationMatrix",10,matrixCallback);
	
	pub = nh.advertise<geometry_msgs::Quaternion>("quaternion",10);
	ros::spin();

	return 0;	
}
