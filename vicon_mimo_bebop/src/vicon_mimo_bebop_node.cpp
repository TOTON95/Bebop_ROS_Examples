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
#define bebop_Offset 1.10							//Distance of Bebop_1 from Bebop_2

//Global Variables
double heading_bebop,heading_mimo;						//Heading of both drones
geometry_msgs::Twist cmd_vel_bebop;						//Velocity message to move the drone
bool b_take_off, b_land = false;						//Bool variables to order the second drone to take-off and land

/////////////// V I C O N - M I M E - B E B O P \\\\\\\\\\\\\\\\\\\\\\\\\

struct object									//Drone objects Bebop (Bebop_1) and Mimo (Bebop_2)
{
	double _posX,_posY,_posZ;						//Position of the drone
	double _errorX,_errorY,_errorZ;						//Position's error
	double _orX,_orY,_orZ,_orW;						//Orientation's error
	double _roll,_pitch,_yaw,_yawRAD;					//Roll, Pitch, Yaw (degrees), YawRAD (radians)
	double _cmdX,_cmdY,_cmdZ,_cmdYAW;					//Commands to move in the X, Y and Z axis, and Yaw
	double rot_cmd_x,rot_cmd_y;						//Position in the rotated matrix
	double _velX,_velY,_velZ,_velYAW;					//Velocity in each of the axis and yaw
	double abs_x,abs_y;							//Absolute position in X and Y
	double angle_res,angle_arc;						//Angle resultant, angle of the arc
}bebop,mimo;
void getBebopPos(const geometry_msgs::TransformStamped::ConstPtr& pos)		//Function to get the Bebop position from the vicon system
{
	bebop._posX = pos->transform.translation.x;				//Position in X
	bebop._posY = pos->transform.translation.y;				//Position in Y
	bebop._posZ = pos->transform.translation.z;				//Position in Z
	bebop._orX = pos->transform.rotation.x;					//Rotation in X
	bebop._orY = pos->transform.rotation.y;					//Rotation in Y
	bebop._orZ = pos->transform.rotation.z;					//Rotation in Z
	bebop._orW = pos->transform.rotation.w;					//Rotation in W

	tf::Quaternion q(pos->transform.rotation.x,pos->transform.rotation.y,pos->transform.rotation.z,pos->transform.rotation.w);
	tf::Matrix3x3 m(q);
	m.getRPY(bebop._roll,bebop._pitch,bebop._yawRAD);			//Function to get the Roll, Pitch and Yaw (Radians)
	bebop._yaw = bebop._yawRAD*(180/PI);					//Converting the radians into degrees
	heading_bebop = bebop._yaw;						//Set the heading into the container
}
void getMimoPos(const geometry_msgs::TransformStamped::ConstPtr& pos)		//Function to get the Mimo position from the vicon system
{
	mimo._posX = pos->transform.translation.x;				//Position in X
	mimo._posY = pos->transform.translation.y;				//Position in Y
	mimo._posZ = pos->transform.translation.z;				//Position in Z
	mimo._orX = pos->transform.rotation.x;					//Rotation in X
	mimo._orY = pos->transform.rotation.y;					//Rotation in Y
	mimo._orZ = pos->transform.rotation.z;					//Rotation in Z
	mimo._orW = pos->transform.rotation.w;					//Rotation in W

	tf::Quaternion q(pos->transform.rotation.x,pos->transform.rotation.y,pos->transform.rotation.z,pos->transform.rotation.w);
	tf::Matrix3x3 m(q);
	m.getRPY(mimo._roll,mimo._pitch,mimo._yawRAD);				//Function to get the Roll, Pitch and Yaw (Radians)
	mimo._yaw = mimo._yawRAD*(180/PI);					//Converting the radians into degrees
	heading_mimo = mimo._yaw;						//Set the heading into the container
}
void takeOffCallBack(const std_msgs::Empty)					//Detects if the "take-off" order had been given
{
	//printf("\n========== T A K E O F F ==========\n");
	b_take_off = true;							//Set the value to true and trigger to Mimo to take-off
}
void landCallback(const std_msgs::Empty)					//Detects if the "land" order had been given
{
	//printf("\n========== L A N D ==========\n");
	b_land = true;								//Set the value to true and trigger to Mimo to land
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"vicon_mimo_bebop");					//Initiates the node 
	ros::NodeHandle n;								//Create the node handle
	ros::Subscriber vicon_mimo, vicon_bebop, takeoff_sub, land_sub;			//Create the subscribers
	ros::Publisher takeoff_bebop, land_bebop, cmd_vel_pub_bebop;			//Create the publishers
	vicon_bebop = n.subscribe("/vicon/Drone_LS/Drone_LS",1000,getBebopPos);		//Initiate the subscriber with the Drone_LS object of the vicon's system
	vicon_mimo = n.subscribe("/vicon/bebop_01/bebop_01",1000,getMimoPos); 		//Initiates the subscriber with the bebop_01 object of the vicon's system.
	takeoff_sub = n.subscribe("/bebop/takeoff",1000,takeOffCallBack);		//Initiates the subscriber to intercept the "take-off" order of the Bebop
	land_sub = n.subscribe("/bebop/land",1000,landCallback);			//Initiates the subscriber to intercept the "land" order of the Bebop
	cmd_vel_pub_bebop = n.advertise<geometry_msgs::Twist>("/bebop1/cmd_vel",1000);	//Initiates the publisher into the topics to command the drone
	takeoff_bebop = n.advertise<std_msgs::Empty>("/bebop1/takeoff",1000);		//Initiates the publisher into the take-off topic of the drone 
	land_bebop = n.advertise<std_msgs::Empty>("/bebop1/land",1000);			//Initiates the publisher into the land topic of the drone

	// PID GAINS
	PID_h pid_hdg,pid_pos,pid_alt;							//Create the PID objects for each action
	pid_hdg.gpro = 0.10;
	pid_hdg.ginteg = 0.0;
	pid_hdg.gder = 0.00002;
	
	pid_pos.gpro = 1.952; // 1.3525 //1.952
	pid_pos.ginteg = 0.0;
	pid_pos.gder = 0.060; //0.0035 //0.050
	
	pid_alt.gpro = 3.5;
	pid_alt.ginteg = 0.0;
        pid_alt.gder = 0.0001;

	double _arc,_arc2,_arc3;							//Arcs

	cmd_vel_bebop.linear.x = 0;							//Set every value to 0
	cmd_vel_bebop.linear.y = 0;
	cmd_vel_bebop.linear.z = 0;

	cmd_vel_bebop.angular.x = 0;	
	cmd_vel_bebop.angular.y = 0;	
	cmd_vel_bebop.angular.z = 0;
	
	ros::Duration(2).sleep();							//Time necesary to setup the publishers
	
	printf("\n========== A C T I V E ==========\n");				//Inform the active state of the program 
	std_msgs::Empty msg_takeoff, msg_land;						//Messages to give the order to take-off and land
	
	ros::Rate r(100);								//Rate of 100 Hz to refresh the program
		
	while(n.ok())									//Run the loop until <Ctrl + C> is pressed
	{	if(b_take_off)								//If take-off is activated then take--off the mimo 
		{
			//printf("\n========== T A K E O F F ==========\n");
			takeoff_bebop.publish(msg_takeoff);				//Take-off the mimo drone
			ros::Duration(4.5).sleep();					//Wait 4.5 seconds until the mimo drone finish the order
			b_take_off = false;						//Restart the variable to use it again
				
		}
		if(b_land)
		{
			//printf("\n========== L A N D [ J S ]==========\n");
			land_bebop.publish(msg_land);					//Land the mimo drone
			b_land = false;							//Restart the variable to use it again
		}
		heading_mimo = constrainAngle(heading_mimo);				//Get the current heading of the bebop drone
		heading_bebop = constrainAngle(heading_bebop);				//Get the current heading of the mimo drone
		_arc = calcArc(heading_bebop,heading_mimo);				//Calculate the current arc of the mimo drone using the bebop's heading as target  
		ros::Duration(0.0001).sleep();						//A little delay to accomplish the order
		_arc2 = calcArc(heading_bebop,heading_mimo);				//Calculate the arc to travel along of it 
		_arc3 = _arc - _arc2;							//Calculate the rest of the arc 
	
		cmd_vel_bebop.angular.z = pid_hdg.mis(_arc3,_arc);			//Data is sent to the heading PID control

		mimo._cmdX = pid_pos.mis(mimo._posX,bebop._posX);			//Data is sent to the position PID control in X
		mimo._cmdY = pid_pos.mis(mimo._posY,bebop._posY + bebop_Offset);	//Data is sent to the position PID control in Y
		mimo.rot_cmd_x = mimo._cmdX*cos(mimo._yawRAD) + mimo._cmdY*sin(mimo._yawRAD);	  //Adapting to the rotated matrix in X
		mimo.rot_cmd_y = mimo._cmdX*sin(mimo._yawRAD) - mimo._cmdY*cos(mimo._yawRAD);	  //Adapting to the rotated matrix in Y
								
		cmd_vel_bebop.linear.x = mimo.rot_cmd_x;				//Set the command data in X
		cmd_vel_bebop.linear.y = -mimo.rot_cmd_y;				//Set the command data in Y
		
		cmd_vel_bebop.linear.z = pid_alt.mis(mimo._posZ,bebop._posZ);		//Set the command data in Z
		cmd_vel_pub_bebop.publish(cmd_vel_bebop);				//Send the command data to the drone
		
		ros::spinOnce();							//Refresh the topics
		r.sleep();								//Set the rate of the program
	}
}
