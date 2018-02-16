#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/LinkStates.h>
#include <geometry_msgs/Pose.h>
//#include <compgx01_msgs/quat2Euler.h>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>

float joint_state_info[5];
geometry_msgs::Pose link_pose[5];
//ros::ServiceClient client;

//row represents the zyx order and column represents the joint (e.g. 6 links has 5 joints)
float euler0[3] ,euler1[3], euler2[3], euler3[3], euler4[3], euler5[3]; 

void quat2Euler(float qx, float qy, float qz, float wq, int q);
void joint_subCallback(const sensor_msgs::JointState::ConstPtr& msg_joint)
{
	// Pass the joint_state message into global variable
	for (int i = 0; i<5 ; i = i + 1)
	{
		joint_state_info[i] = msg_joint->position[i];
	}

}	

void link_subCallback(const gazebo_msgs::LinkStates::ConstPtr& msg_link)
{
	// Store the pose from gazebo/link_states
	int link_no = msg_link->name.size();

	for (int i = 0; i<link_no-1 ; i = i + 1)
	{
		link_pose[i] = msg_link->pose[i+1];
	}


}
// this function solve the problem of failed service request to convert quat2Euler 
void quat2Euler(float qx, float qy, float qz, float qw, int count)
{
	float q[4] = {qw,qx,qy,qz};
	// Mathematical manipulation
	float x1 = 2.0 * (q[0]*q[1] + q[2]*q[3]);
	float x2 = 1 - 2.0*(q[1]*q[1] + q[2]*q[2]);
	float roll = atan2(x1,x2);

	float y1 = 2.0 * (q[0]*q[2] - q[3]*q[1]);
	float pitch = 0;
	if (fabs(y1) >= 1)
		pitch = copysign(M_PI/2, y1); //use 90degrees if out of range
	else
		pitch = asin(y1);

	float z1 = 2.0 * (q[0]*q[3] + q[1]*q[2]);
	float z2 = 1 - 2.0*(q[2]*q[2] + q[3]*q[3]);
	float yaw = atan2(z1,z2);
	
	if (count==0){
		euler0[0]=roll;
		euler0[1]=pitch;
		euler0[2]=yaw;
	}
	if (count==1){
		euler1[0]=roll;
		euler1[1]=pitch;
		euler1[2]=yaw;
	}
	if (count==2){
		euler2[0]=roll;
		euler2[1]=pitch;
		euler2[2]=yaw;
	}
	if (count==3){
		euler3[0]=roll;
		euler3[1]=pitch;
		euler3[2]=yaw;
	}
	if (count==4){
		euler4[0]=roll;
		euler4[1]=pitch;
		euler4[2]=yaw;
	}
	if (count==5){
		euler5[0]=roll;
		euler5[1]=pitch;
		euler5[2]=yaw;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc,argv,"forwardKinematics_dh");
	ros::NodeHandle n;

	ros::Subscriber joint_sub = n.subscribe("joint_states",1000,joint_subCallback);
	ros::Subscriber link_sub = n.subscribe("gazebo/link_states",1000,link_subCallback);

	//client = n.serviceClient<compgx01_msgs::quat2Euler>("quat2Euler_converter");

	//ros::spin();
	ros::Rate r(10);

	while (ros::ok())
	{

	// Compute DH Variables for the 5 joints
	int joint_no = 5;
	float d[joint_no], theta[joint_no];
	float a[joint_no], alpha[joint_no];
	
	// custom function to convert quaternion to euler
	for (int i = 0; i<=joint_no ; i=i+1)
	{
		quat2Euler(link_pose[i].orientation.x,link_pose[i].orientation.y,link_pose[i].orientation.z,link_pose[i].orientation.w,i);

	}

/*	
//	this is not being implemented because there is an error of service request fault
	// Convert quarternion to Euler zyx
	compgx01_msgs::quat2Euler srv;
	// link 0
	srv.request.input = link_pose[0].orientation;
	if (client.call(srv))
	{
		euler0[0]=srv.response.output.angle[0]; // x
		euler0[1]=srv.response.output.angle[1]; // y
		euler0[2]=srv.response.output.angle[2]; // z
	}
	else
	{
		ROS_ERROR("Failed to call service quat2Euler, remember to run the service server coursework_1_node");
	}

	// link 1
	srv.request.input = link_pose[1].orientation;
	if (client.call(srv))
	{
		euler1[0]=srv.response.output.angle[0]; // x
		euler1[1]=srv.response.output.angle[1]; // y
		euler1[2]=srv.response.output.angle[2]; // z
	}
	else
	{
		ROS_ERROR("Failed to call service quat2Euler, remember to run the service server coursework_1_node");
	}

	// link 2
	srv.request.input = link_pose[2].orientation;
	if (client.call(srv))
	{
		euler2[0]=srv.response.output.angle[0]; // x
		euler2[1]=srv.response.output.angle[1]; // y
		euler2[2]=srv.response.output.angle[2]; // z
	}
	else
	{
		ROS_ERROR("Failed to call service quat2Euler, remember to run the service server coursework_1_node");
	}

	// link 3
	srv.request.input = link_pose[3].orientation;
	if (client.call(srv))
	{
		euler3[0]=srv.response.output.angle[0]; // x
		euler3[1]=srv.response.output.angle[1]; // y
		euler3[2]=srv.response.output.angle[2]; // z
	}
	else
	{
		ROS_ERROR("Failed to call service quat2Euler, remember to run the service server coursework_1_node");
	}

	// link 4
	srv.request.input = link_pose[4].orientation;
	if (client.call(srv))
	{
		euler4[0]=srv.response.output.angle[0]; // x
		euler4[1]=srv.response.output.angle[1]; // y
		euler4[2]=srv.response.output.angle[2]; // z
	}
	else
	{
		ROS_ERROR("Failed to call service quat2Euler, remember to run the service server coursework_1_node");
	}

	// link 5
	srv.request.input = link_pose[5].orientation;
	if (client.call(srv))
	{
		euler5[0]=srv.response.output.angle[0]; // x
		euler5[1]=srv.response.output.angle[1]; // y
		euler5[2]=srv.response.output.angle[2]; // z
	}
	else
	{
		ROS_ERROR("Failed to call service quat2Euler, remember to run the service server coursework_1_node");
	}
*/

	// these are the dh variables
	// At joint 1
	d[0] 	 = link_pose[1].position.z - link_pose[0].position.z;
	theta[0] = euler1[2]-euler0[2] - joint_state_info[0];
	a[0]     = link_pose[1].position.x - link_pose[0].position.x;
	alpha[0] = euler1[0]-euler0[0];

	ROS_INFO("%f %f %f %f\n",d[0],theta[0],a[0],alpha[0]);

	// At joint 2
	d[1] 	 = link_pose[2].position.z - link_pose[1].position.z;
	theta[1] = euler2[2]-euler1[2];
	a[1]     = link_pose[2].position.x - link_pose[1].position.x;
	alpha[1] = euler2[0]-euler1[0] - M_PI/2; //rotate around x-axis by 90degrees to change the rotation axis into Z
	ROS_INFO("%f %f %f %f\n",d[1],theta[1],a[1],alpha[1]);

	// At joint 3
	d[2] 	 = link_pose[3].position.y - link_pose[2].position.y;
	theta[2] = euler2[1]-euler1[1] - joint_state_info[1] - M_PI/2; // to match the axis
	a[2]     = link_pose[3].position.z - link_pose[3].position.z;
	alpha[2] = euler3[2]-euler2[2];
	ROS_INFO("%f %f %f %f\n",d[2],theta[2],a[2],alpha[2]);

	// At joint 4
	d[3] 	 = link_pose[4].position.y - link_pose[3].position.y;
	theta[3] = euler3[2]-euler2[2] - joint_state_info[2];
	a[3]     = link_pose[4].position.z - link_pose[3].position.z;
	alpha[3] = euler4[2]-euler3[2];
	ROS_INFO("%f %f %f %f\n",d[3],theta[3],a[3],alpha[3]);

	// At joint 5
	d[4] 	 = link_pose[5].position.y - link_pose[4].position.y;
	theta[4] = euler4[2]-euler3[2] - joint_state_info[3] + M_PI/2;
	a[4]     = link_pose[5].position.x - link_pose[4].position.x;
	alpha[4] = euler5[0]-euler4[0] - M_PI/2;


	ROS_INFO("%f %f %f %f\n\n",d[4],theta[4],a[4],alpha[4]);

	// tf broadcaster
	// the tf is slightly different from the youbot as the dh variable does not allow rotation or translation
	// around y -axis, however the transformation in ROS allows translation and rotation along 3 axis
	// also, since the transformation are not commutative, the sequential order on how ROS applies the transform is 
	// unknown. Hence it shows a little strange result. 
	static tf2_ros::TransformBroadcaster br[5];
	geometry_msgs::TransformStamped transformStamped[5];

	transformStamped[0].header.stamp = ros::Time::now();
	transformStamped[0].header.frame_id = "armLink_0";
	transformStamped[0].child_frame_id = "armLink_1";
	transformStamped[0].transform.translation.x = a[0];
	transformStamped[0].transform.translation.y = 0;
	transformStamped[0].transform.translation.z = d[0];
	tf2::Quaternion q;
	q.setRPY(alpha[0],0,theta[0]);
	transformStamped[0].transform.rotation.x = q.x();	
	transformStamped[0].transform.rotation.y = q.y();
	transformStamped[0].transform.rotation.z = q.z();	
	transformStamped[0].transform.rotation.w = q.w();

	br[0].sendTransform(transformStamped[0]);

	transformStamped[1].header.stamp = ros::Time::now();
	transformStamped[1].header.frame_id = "armLink_1";
	transformStamped[1].child_frame_id = "armLink_2";
	transformStamped[1].transform.translation.x = -a[1];
	transformStamped[1].transform.translation.y = 0;
	transformStamped[1].transform.translation.z = d[1];
	q.setRPY(alpha[1]+M_PI/2,theta[1]+theta[2]+M_PI/2,0);
	transformStamped[1].transform.rotation.x = q.x();	
	transformStamped[1].transform.rotation.y = q.y();
	transformStamped[1].transform.rotation.z = q.z();	
	transformStamped[1].transform.rotation.w = q.w();

	br[1].sendTransform(transformStamped[1]);

	transformStamped[2].header.stamp = ros::Time::now();
	transformStamped[2].header.frame_id = "armLink_2";
	transformStamped[2].child_frame_id = "armLink_3";
	transformStamped[2].transform.translation.x = a[2];
	transformStamped[2].transform.translation.y = 0;
	transformStamped[2].transform.translation.z = d[2];
	q.setRPY(alpha[2],theta[3],0);
	transformStamped[2].transform.rotation.x = q.x();	
	transformStamped[2].transform.rotation.y = q.y();
	transformStamped[2].transform.rotation.z = q.z();	
	transformStamped[2].transform.rotation.w = q.w();

	br[2].sendTransform(transformStamped[2]);

	transformStamped[3].header.stamp = ros::Time::now();
	transformStamped[3].header.frame_id = "armLink_3";
	transformStamped[3].child_frame_id = "armLink_4";
	transformStamped[3].transform.translation.x = a[3];
	transformStamped[3].transform.translation.y = 0;
	transformStamped[3].transform.translation.z = d[3];
	q.setRPY(alpha[3],theta[4]-M_PI/2,0);
	transformStamped[3].transform.rotation.x = q.x();	
	transformStamped[3].transform.rotation.y = q.y();
	transformStamped[3].transform.rotation.z = q.z();	
	transformStamped[3].transform.rotation.w = q.w();

	br[3].sendTransform(transformStamped[3]);

	transformStamped[4].header.stamp = ros::Time::now();
	transformStamped[4].header.frame_id = "armLink_4";
	transformStamped[4].child_frame_id = "armLink_5";
	transformStamped[4].transform.translation.x = a[4];
	transformStamped[4].transform.translation.y = 0;
	transformStamped[4].transform.translation.z = d[4];
	q.setRPY(alpha[4],0,theta[4]);
	transformStamped[4].transform.rotation.x = q.x();	
	transformStamped[4].transform.rotation.y = q.y();
	transformStamped[4].transform.rotation.z = q.z();	
	transformStamped[4].transform.rotation.w = q.w();

	br[4].sendTransform(transformStamped[4]);
	
	ros::spinOnce();
	r.sleep();

	}
	return 0;
}
