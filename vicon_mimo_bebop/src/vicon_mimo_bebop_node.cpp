//Coded by Alexis Guijarro

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Point.h>

#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>

#include <vicon_mimo_bebop/PID_ROS.h>
#include <vicon_mimo_bebop/ArcDrone.h>

#include <stdio.h>
#include <cmath>
#include <math.h>

#define PI 3.141592653589793238462
#define bebop_Offset 1.10

//Global Variables
double heading_bebop,heading_mimo;
geometry_msgs::Twist cmd_vel_bebop;
bool b_take_off, b_land = false;

/////////////// V I C O N - M I M E - B E B O P \\\\\\\\\\\\\\\\\\\\\\\\\

struct object
{
	double _posX,_posY,_posZ;
	double _errorX,_errorY,_errorZ;
	double _orX,_orY,_orZ,_orW;
	double _roll,_pitch,_yaw,_yawRAD;
	double _cmdX,_cmdY,_cmdZ,_cmdYAW;
	double rot_cmd_x,rot_cmd_y;
	double _velX,_velY,_velZ,_velYAW;
	double abs_x,abs_y;
	double angle_res,angle_arc;
}bebop,mimo;
void getBebopPos(const geometry_msgs::TransformStamped::ConstPtr& pos)
{
	bebop._posX = pos->transform.translation.x;
	bebop._posY = pos->transform.translation.y;
	bebop._posZ = pos->transform.translation.z;
	bebop._orX = pos->transform.rotation.x;
	bebop._orY = pos->transform.rotation.y;
	bebop._orZ = pos->transform.rotation.z;
	bebop._orW = pos->transform.rotation.w;

	tf::Quaternion q(pos->transform.rotation.x,pos->transform.rotation.y,pos->transform.rotation.z,pos->transform.rotation.w);
	tf::Matrix3x3 m(q);
	m.getRPY(bebop._roll,bebop._pitch,bebop._yawRAD);
	bebop._yaw = bebop._yawRAD*(180/PI);
	heading_bebop = bebop._yaw;
}
void getMimoPos(const geometry_msgs::TransformStamped::ConstPtr& pos)
{
	mimo._posX = pos->transform.translation.x;
	mimo._posY = pos->transform.translation.y;
	mimo._posZ = pos->transform.translation.z;
	mimo._orX = pos->transform.rotation.x;
	mimo._orY = pos->transform.rotation.y;
	mimo._orZ = pos->transform.rotation.z;
	mimo._orW = pos->transform.rotation.w;

	tf::Quaternion q(pos->transform.rotation.x,pos->transform.rotation.y,pos->transform.rotation.z,pos->transform.rotation.w);
	tf::Matrix3x3 m(q);
	m.getRPY(mimo._roll,mimo._pitch,mimo._yawRAD);
	mimo._yaw = mimo._yawRAD*(180/PI);
	heading_mimo = mimo._yaw;
}
void takeOffCallBack(const std_msgs::Empty)
{
	//printf("\n========== T A K E O F F ==========\n");
	b_take_off = true;
}
void landCallback(const std_msgs::Empty)
{
	//printf("\n========== L A N D ==========\n");
	b_land = true;
}
int main(int argc, char** argv)
{
	ros::init(argc,argv,"bebop_vicon_wand");
	ros::NodeHandle n;
	ros::Subscriber vicon_mimo, vicon_bebop, takeoff_sub, land_sub;
	ros::Publisher takeoff_bebop, land_bebop, cmd_vel_pub_bebop;
	vicon_bebop = n.subscribe("/vicon/Drone_LS/Drone_LS",1000,getBebopPos);
	vicon_mimo = n.subscribe("/vicon/bebop_01/bebop_01",1000,getMimoPos);
	takeoff_sub = n.subscribe("/bebop/takeoff",1000,takeOffCallBack);
	land_sub = n.subscribe("/bebop/land",1000,landCallback);
	cmd_vel_pub_bebop = n.advertise<geometry_msgs::Twist>("/bebop1/cmd_vel",1000);
	takeoff_bebop = n.advertise<std_msgs::Empty>("/bebop1/takeoff",1000);
	land_bebop = n.advertise<std_msgs::Empty>("/bebop1/land",1000);

	// PID GAINS
	PID_h pid_hdg,pid_pos,pid_alt;
	pid_hdg.gpro = 0.10;
	pid_hdg.ginteg = 0.0;
	pid_hdg.gder = 0.00002;
	
	pid_pos.gpro = 1.952; // 1.3525 //1.952
	pid_pos.ginteg = 0.0;
	pid_pos.gder = 0.060; //0.0035 //0.050
	
	pid_alt.gpro = 3.5;
	pid_alt.ginteg = 0.0;
        pid_alt.gder = 0.0001;

	double _arc,_arc2,_arc3;

	cmd_vel_bebop.linear.x = 0;
	cmd_vel_bebop.linear.y = 0;
	cmd_vel_bebop.linear.z = 0;

	cmd_vel_bebop.angular.x = 0;	
	cmd_vel_bebop.angular.y = 0;	
	cmd_vel_bebop.angular.z = 0;
	
	//Time necesary to setup the publishers
	ros::Duration(2).sleep();
	
	printf("\n========== A C T I V E ==========\n");
	std_msgs::Empty msg_takeoff, msg_land;
	
	ros::Rate r(100);
	
	while(n.ok())
	{	if(b_take_off)
		{
			//printf("\n========== T A K E O F F ==========\n");
			takeoff_bebop.publish(msg_takeoff);
			ros::Duration(4.5).sleep();
			b_take_off = false;
				
		}
		if(b_land)
		{
			//printf("\n========== L A N D [ J S ]==========\n");
			land_bebop.publish(msg_land);
			b_land = false;	
		}
		heading_mimo = constrainAngle(heading_mimo);
		heading_bebop = constrainAngle(heading_bebop);
		_arc = calcArc(heading_bebop,heading_mimo);
		ros::Duration(0.0001).sleep();
		_arc2 = calcArc(heading_bebop,heading_mimo);
		_arc3 = _arc - _arc2;

		cmd_vel_bebop.angular.z = pid_hdg.mis(_arc3,_arc);

		mimo._cmdX = pid_pos.mis(mimo._posX,bebop._posX);
		mimo._cmdY = pid_pos.mis(mimo._posY,bebop._posY + bebop_Offset);
		mimo.rot_cmd_x = mimo._cmdX*cos(mimo._yawRAD) + mimo._cmdY*sin(mimo._yawRAD);
		mimo.rot_cmd_y = mimo._cmdX*sin(mimo._yawRAD) - mimo._cmdY*cos(mimo._yawRAD);
								
		cmd_vel_bebop.linear.x = mimo.rot_cmd_x;
		cmd_vel_bebop.linear.y = -mimo.rot_cmd_y;
		
		cmd_vel_bebop.linear.z = pid_alt.mis(mimo._posZ,bebop._posZ);
		cmd_vel_pub_bebop.publish(cmd_vel_bebop);
		
		ros::spinOnce();
		r.sleep();
	}
}
