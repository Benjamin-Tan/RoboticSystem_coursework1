#include "ros/ros.h"
#include "compgx01_msgs/quat2Euler.h"
#include "compgx01_msgs/quat2AngAxis.h"
#include "compgx01_msgs/rotmat2Quat.h"

bool convert_quat2Euler(compgx01_msgs::quat2Euler::Request  &req,
         compgx01_msgs::quat2Euler::Response &res)
{
    //TO DO:
    //Put your rotation representations converter here.....
	
	double q[4] = {req.input.w, req.input.x, req.input.y, req.input.z};

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

	res.output.order = "ZYX, (roll,pitch,yaw)";
	
	res.output.angle[0] = roll;
	res.output.angle[1] = pitch;
	res.output.angle[2] = yaw;

    return true;
}

// Build your service first and uncomment these lines
bool convert_quat2AngAxis(compgx01_msgs::quat2AngAxis::Request  &req,
                        compgx01_msgs::quat2AngAxis::Response &res)
{
    //TO DO:
    //Put your rotation representations converter here.....
	double q[4] = {req.input.w, req.input.x, req.input.y, req.input.z};

	// Mathematical manipulation

	double denominator = sqrt(1-q[0]*q[0]);
	res.output.angle = 2*acos(q[0]);
	if (denominator < 0.001)
	{
		res.output.axis.x = q[1];
		res.output.axis.y = q[2];
		res.output.axis.z = q[3];
		
	}
	else
	{
		res.output.axis.x = q[1] / denominator;
		res.output.axis.y = q[2] / denominator;
		res.output.axis.z = q[3] / denominator;
	}	
    return true;
}

// Build your service first and uncomment these lines
bool convert_rotmat2Quat(compgx01_msgs::rotmat2Quat::Request  &req,
                        compgx01_msgs::rotmat2Quat::Response &res)
{
    //TO DO:
    //Put your rotation representations converter here.....
	double x,y,z,w;
	int matrixSize = 9;
	double r[matrixSize];

	for (int a=0 ; a < 3; a = a + 1){
		r[a]=req.input.first_row[a];
	}
	for (int b=0 ; b < 3; b = b + 1){
		r[b+3]=req.input.second_row[b];
	}
	for (int c=0 ; c < 3; c = c + 1){
		r[c+6]=req.input.third_row[c];
	}

	// Mathematical manipulation
	
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

	res.output.x = x;
	res.output.y = y;
	res.output.z = z;
	res.output.w = w;
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "quat2Euler_server");
    ros::NodeHandle n;

    ros::ServiceServer service_quat2Euler = n.advertiseService("quat2Euler_converter", convert_quat2Euler);
    ros::ServiceServer service_quat2AngleAxis = n.advertiseService("quat2AngAxis_converter", convert_quat2AngAxis);
    ros::ServiceServer service_rotMat2quat = n.advertiseService("rotmat2Quat_converter", convert_rotmat2Quat);

    ROS_INFO("Ready to convert from a quaternion to Euler angle representation.");
    ros::spin();

    return 0;
}
