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
#define Wand_Offset 1.45							//Offset Wand (Bebop Target)

// Global Variables
double wand_offset_x = 0.0;							//Wand's Offset in X
double wand_offset_y = 0.0;							//Wand's Offset in Y
double heading;									//Drone's heading
int btn_emergency;								//Emergency's Button to stop routine
geometry_msgs::Twist cmd_vel_bebop;						//Command Message
double aux_X,aux_Y;								//Auxiliar variables


/////////////// V I C O N - W A N D - B E B O P \\\\\\\\\\\\\\\\\\\\\\\\\

struct object									//Structure that describes the properties of the drone
{
	double _posX,_posY,_posZ;						//Position of the drone
	double _errorX,_errorY,_errorZ;						//Error of the position of the drone
	double _orX,_orY,_orZ,_orW;						//Orientation of the drone
	double _roll,_pitch,_yaw,_yawRAD;					//Roll,Pitch,Yaw (degrees), Yaw (radians)
	double _cmdX,_cmdY,_cmdZ,_cmdYAW;					//Command values 
	double rot_cmd_x,rot_cmd_y;						//Position in the rotated matrix
	double _velX,_velY,_velZ,_velYAW;					//Velocities
	double abs_x,abs_y;							//Absolute position in X and Y
	double angle_res,angle_arc;						//Angle resultant, angle of the arc
}bebop,wand;

void getJoyState(const sensor_msgs::Joy::ConstPtr& js)				//Function to obtain the data from the emergency button of the joystick
{
	btn_emergency = js->buttons[0];
}
void getWandPos(const geometry_msgs::TransformStamped::ConstPtr& pos)		//Function to obtain the position of the wand from the vicon system
{
	wand._posX = pos->transform.translation.x;				//Position in X
	wand._posY = pos->transform.translation.y;				//Position in Y
	wand._posZ = pos->transform.translation.z;				//Position in Z
	wand._orX = pos->transform.rotation.x;					//Rotation in X
	wand._orY = pos->transform.rotation.y;					//Rotation in Y
	wand._orZ = pos->transform.rotation.z;					//Rotation in Z
	wand._orW = pos->transform.rotation.w;					//Rotation in W

	tf::Quaternion q(pos->transform.rotation.x,pos->transform.rotation.y,pos->transform.rotation.z,pos->transform.rotation.w);
	tf::Matrix3x3 m(q);
	m.getRPY(wand._roll,wand._pitch,wand._yawRAD);				//Get the Roll, Pitch, Yaw (Radians)
	wand._yaw = wand._yawRAD*(180/PI);					//Convert the Yaw (Radians) into Yaw (Degrees)
	wand._yaw = constrainAngle(wand._yaw);					//Set the heading of the drone
}
void getBebopPos(const geometry_msgs::TransformStamped::ConstPtr& pos)		//Function to obtain the position of the bebop from the vicon system
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
	m.getRPY(bebop._roll,bebop._pitch,bebop._yawRAD);			//Get the Roll, Pitch, Yaw (Radians)
	bebop._yaw = bebop._yawRAD*(180/PI);					//Convert the Yaw (Radians) into Yaw (Degrees)
	heading = bebop._yaw;							//Set the heading of the drone
}
void setWandOffset(void)
{
	//wand_offset_x = (Wand_Offset * cos(wand._yaw)) ;
	//wand_offset_y = (Wand_Offset * sin(wand._yaw)) ;
	wand_offset_x = cos(wand._yawRAD) + wand._posX;				//Get the position of the target using the wand's offset in X
	wand_offset_y = sin(wand._yawRAD) + wand._posY;				//Get the position of the target using the wand's offset in Y
}
int main(int argc, char** argv)
{
	ros::init(argc,argv,"bebop_vicon_wand");				//Initiates the node
	ros::NodeHandle n;							//Creates the node handler
	ros::Subscriber vicon_wand, joy_sub, vicon_bebop;			//Creates the subscribers of the joystick and the vicon system
	ros::Publisher takeoff_bebop, land_bebop, cmd_vel_pub_bebop;		//Creates the publisher of the take-off, land and movement command
	joy_sub = n.subscribe("/joy",1000,getJoyState);				//Initiates the Joystick Subscriber
	vicon_wand = n.subscribe("/vicon/Wand_LS/Wand_LS",1000,getWandPos);	//Initiates the Wand Subscriber
	vicon_bebop = n.subscribe("/vicon/Drone_LS/Drone_LS",1000,getBebopPos);		//Initiates the Drone_LS Subscriber
	cmd_vel_pub_bebop = n.advertise<geometry_msgs::Twist>("/bebop/cmd_vel",1000);	//Initiates the Command publisher into  the topics
	takeoff_bebop = n.advertise<std_msgs::Empty>("/bebop/takeoff",1000);	//Initiates the Take-Off publisher into the topics
	land_bebop = n.advertise<std_msgs::Empty>("/bebop/land",1000);		//Initiates the Land publisher into the topics

	//Video Recording, not fully tested 
	ros::Publisher video;
	video = n.advertise<std_msgs::Bool>("/bebop/record",1);
	std_msgs::Bool record;

	// PID GAINS
	PID_h pid_hdg,pid_pos,pid_alt;						//Create a pid object for heading, position and altitude
	pid_hdg.gpro = 0.10;							//Setting gains
	pid_hdg.ginteg = 0.0;
	pid_hdg.gder = 0.00002;
	
	pid_pos.gpro = 1.952; // 1.3525
	pid_pos.ginteg = 0.0;
	pid_pos.gder = 0.050; //0.0035
	
	pid_alt.gpro = 3.5;
	pid_alt.ginteg = 0.0;
        pid_alt.gder = 0.0001;
	
	std_msgs::Empty msg_takeoff, msg_land;					//Message to command the drone to take-off and land
	double _arc,_arc2,_arc3;						//Arcs for the heading 

	cmd_vel_bebop.linear.x = 0;						//Set every value to 0
	cmd_vel_bebop.linear.y = 0;
	cmd_vel_bebop.linear.z = 0;

	cmd_vel_bebop.angular.x = 0;	
	cmd_vel_bebop.angular.y = 0;	
	cmd_vel_bebop.angular.z = 0;

	ros::spinOnce();							//Refresh the topics

	double hdg_target = 0;							//Initial target of the heading 
	heading = constrainAngle(heading);					//Get the current heading
	_arc = calcArc(hdg_target,heading);					//Calculate the arc using as reference the initial target
	
	ros::Duration(2).sleep();						//Time necesary to setup the publishers

	record.data = true;
	video.publish(record);
	
	printf("\n========== T A K E O F F ==========\n");
	takeoff_bebop.publish(msg_takeoff);					//Take-off
	ros::Duration(4.5).sleep();						//Wait until the drone is flying
	
	//Stable Drone
	cmd_vel_pub_bebop.publish(cmd_vel_bebop);				//Set the drone to hover
	ros::Duration(0.5).sleep();						//Wait until the order is accomplished
	
	ros::Rate r(100);							//Setting the program to 100 Hz
	
	while(n.ok())								//Execute the program until <Ctrl + C> is pressed
	{
		if(btn_emergency)						//If the joystick's button is pressed land the drone and finish the program
		{
			ros::Duration(0.525).sleep();
			printf("\n========== L A N D [ J S ]==========\n");
			land_bebop.publish(msg_land);				//Land the drone
			record.data = false;
			video.publish(record);
			break;
		}
		setWandOffset();						//Calculate the target of the drone
		hdg_target = wand._yaw + 180;					//Make the drone face the wand
		heading = constrainAngle(heading);				//Get current heading
	        _arc2 = calcArc(hdg_target,heading);				//Calculate the arc to travel along of it 
		_arc3 = _arc - _arc2;						//Calculate the rest of the arc 
		cmd_vel_bebop.angular.z = pid_hdg.mis(_arc3,_arc);		//Data is sent to the heading PID control

		//Data calculated using the target of the wand is sent to Position PID control in X
		bebop._cmdX = pid_pos.mis(bebop._posX,wand_offset_x);
		//Data calculated using the target of the wand is sent to Position PID control in X		
		bebop._cmdY = pid_pos.mis(bebop._posY,wand_offset_y);		
		bebop.rot_cmd_x = bebop._cmdX*cos(bebop._yawRAD) + bebop._cmdY*sin(bebop._yawRAD);	//Adapting to the rotated matrix in X
		bebop.rot_cmd_y = bebop._cmdX*sin(bebop._yawRAD) - bebop._cmdY*cos(bebop._yawRAD);	//Adapting to the rotated matrix in Y
					
					
		cmd_vel_bebop.linear.x = bebop.rot_cmd_x;			//Set the command data in X
		cmd_vel_bebop.linear.y = -bebop.rot_cmd_y;			//Set the command data in Y

		printf("\nX: %lf Y: %lf Yaw: %lf NX: %lf NY: %lf \n ",wand._posX,wand._posY,wand._yaw, wand_offset_x, wand_offset_y);
		cmd_vel_bebop.linear.z = pid_alt.mis(bebop._posZ,1.10);		//Set the Altitude PID to 1.10 mts
		cmd_vel_pub_bebop.publish(cmd_vel_bebop);			//Set the command to move the drone
		/*if(wand._posZ < 0.15)
		{
			ros::Duration(0.525).sleep();
			printf("\n========== L A N D [ W A N D ]==========\n");
			land_bebop.publish(msg_land);
			break;
		}*/
		ros::spinOnce();						//Refresh the topics
		r.sleep();							//Set the rate of the program
	}
	ros::Duration(2).sleep();						//Wait 2 seconds to let the program finish some process
	ros::spinOnce();							//Refresh the topics
	if(bebop._posZ > 0.60)							//If the drone still in the air, land it
	{
		ros::Duration(0.525).sleep();
		printf("\n========== L A N D [ A L T ]==========\n");
		land_bebop.publish(msg_land);					//Land
	}
	record.data = false;
	video.publish(record);
	return 0;								//End the program
}







