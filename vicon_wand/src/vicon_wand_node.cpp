//Coded by Alexis Guijarro

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>

#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>

#include <vicon_wand/PID_ROS.h>
#include <vicon_wand/ArcDrone.h>

#include <stdio.h>
#include <cmath>
#include <math.h>

#define PI 3.141592653589793238462
#define Wand_Offset 1.45

// Global Variables
double wand_offset_x = 0.0;
double wand_offset_y = 0.0;
double heading;
int btn_emergency;
geometry_msgs::Twist cmd_vel_bebop;
double aux_X,aux_Y;


/////////////// V I C O N - W A N D - B E B O P \\\\\\\\\\\\\\\\\\\\\\\\\

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
}bebop,wand;

void getJoyState(const sensor_msgs::Joy::ConstPtr& js)
{
	btn_emergency = js->buttons[0];
}
void getWandPos(const geometry_msgs::TransformStamped::ConstPtr& pos)
{
	wand._posX = pos->transform.translation.x;
	wand._posY = pos->transform.translation.y;
	wand._posZ = pos->transform.translation.z;
	wand._orX = pos->transform.rotation.x;
	wand._orY = pos->transform.rotation.y;
	wand._orZ = pos->transform.rotation.z;
	wand._orW = pos->transform.rotation.w;

	tf::Quaternion q(pos->transform.rotation.x,pos->transform.rotation.y,pos->transform.rotation.z,pos->transform.rotation.w);
	tf::Matrix3x3 m(q);
	m.getRPY(wand._roll,wand._pitch,wand._yawRAD);
	wand._yaw = wand._yawRAD*(180/PI);
	wand._yaw = constrainAngle(wand._yaw);
}
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
	heading = bebop._yaw;
}
void setWandOffset(void)
{
	//wand_offset_x = (Wand_Offset * cos(wand._yaw)) ;
	//wand_offset_y = (Wand_Offset * sin(wand._yaw)) ;
	wand_offset_x = cos(wand._yawRAD) + wand._posX;
	wand_offset_y = sin(wand._yawRAD) + wand._posY;
}
int main(int argc, char** argv)
{
	ros::init(argc,argv,"bebop_vicon_wand");
	ros::NodeHandle n;
	ros::Subscriber vicon_wand, joy_sub, vicon_bebop;
	ros::Publisher takeoff_bebop, land_bebop, cmd_vel_pub_bebop;
	joy_sub = n.subscribe("/joy",1000,getJoyState);
	vicon_wand = n.subscribe("/vicon/Wand_LS/Wand_LS",1000,getWandPos);
	vicon_bebop = n.subscribe("/vicon/Drone_LS/Drone_LS",1000,getBebopPos);
	cmd_vel_pub_bebop = n.advertise<geometry_msgs::Twist>("/bebop/cmd_vel",1000);
	takeoff_bebop = n.advertise<std_msgs::Empty>("/bebop/takeoff",1000);
	land_bebop = n.advertise<std_msgs::Empty>("/bebop/land",1000);

	//Video Recording, not fully tested 
	ros::Publisher video;
	video = n.advertise<std_msgs::Bool>("/bebop/record",1);
	std_msgs::Bool record;

	// PID GAINS
	PID_h pid_hdg,pid_pos,pid_alt;
	pid_hdg.gpro = 0.10;
	pid_hdg.ginteg = 0.0;
	pid_hdg.gder = 0.00002;
	
	pid_pos.gpro = 1.952; // 1.3525
	pid_pos.ginteg = 0.0;
	pid_pos.gder = 0.050; //0.0035
	
	pid_alt.gpro = 3.5;
	pid_alt.ginteg = 0.0;
        pid_alt.gder = 0.0001;
	
	std_msgs::Empty msg_takeoff, msg_land;
	double _arc,_arc2,_arc3;

	cmd_vel_bebop.linear.x = 0;
	cmd_vel_bebop.linear.y = 0;
	cmd_vel_bebop.linear.z = 0;

	cmd_vel_bebop.angular.x = 0;	
	cmd_vel_bebop.angular.y = 0;	
	cmd_vel_bebop.angular.z = 0;

	ros::spinOnce();

	double hdg_target = 0;
	heading = constrainAngle(heading);
	_arc = calcArc(hdg_target,heading);
	
	//Time necesary to setup the publishers
	ros::Duration(2).sleep();

	record.data = true;
	video.publish(record);
	
	printf("\n========== T A K E O F F ==========\n");
	takeoff_bebop.publish(msg_takeoff);
	ros::Duration(4.5).sleep();
	
	//Stable Drone
	cmd_vel_pub_bebop.publish(cmd_vel_bebop);
	ros::Duration(0.5).sleep();
	
	ros::Rate r(100);
	
	while(n.ok())
	{
		if(btn_emergency)
		{
			ros::Duration(0.525).sleep();
			printf("\n========== L A N D [ J S ]==========\n");
			land_bebop.publish(msg_land);
			record.data = false;
			video.publish(record);
			break;
		}
		setWandOffset();
		hdg_target = wand._yaw + 180;
		heading = constrainAngle(heading);
	        _arc2 = calcArc(hdg_target,heading);
		_arc3 = _arc - _arc2;
		cmd_vel_bebop.angular.z = pid_hdg.mis(_arc3,_arc);
		bebop._cmdX = pid_pos.mis(bebop._posX,wand_offset_x);
		bebop._cmdY = pid_pos.mis(bebop._posY,wand_offset_y);
		bebop.rot_cmd_x = bebop._cmdX*cos(bebop._yawRAD) + bebop._cmdY*sin(bebop._yawRAD);
		bebop.rot_cmd_y = bebop._cmdX*sin(bebop._yawRAD) - bebop._cmdY*cos(bebop._yawRAD);
					
					
		cmd_vel_bebop.linear.x = bebop.rot_cmd_x;
		cmd_vel_bebop.linear.y = -bebop.rot_cmd_y;

		printf("\nX: %lf Y: %lf Yaw: %lf NX: %lf NY: %lf \n ",wand._posX,wand._posY,wand._yaw, wand_offset_x, wand_offset_y);
		cmd_vel_bebop.linear.z = pid_alt.mis(bebop._posZ,1.10);
		cmd_vel_pub_bebop.publish(cmd_vel_bebop);
		/*if(wand._posZ < 0.15)
		{
			ros::Duration(0.525).sleep();
			printf("\n========== L A N D [ W A N D ]==========\n");
			land_bebop.publish(msg_land);
			break;
		}*/
		ros::spinOnce();
		r.sleep();
	}
	ros::Duration(2).sleep();
	ros::spinOnce();
	if(bebop._posZ > 0.60)
	{
		ros::Duration(0.525).sleep();
		printf("\n========== L A N D [ A L T ]==========\n");
		land_bebop.publish(msg_land);
	}
	record.data = false;
	video.publish(record);
	return 0;
}







