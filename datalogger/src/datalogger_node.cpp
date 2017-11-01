//Coded by Alexis Guijarro

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <datalogger/Altitude_datalogger.h>
#include <tf/transform_datatypes.h>
#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>

#define PI 3.141592653589793238462 		//PI

using namespace std;				//Namespace to avoid unnecesary coding		

nav_msgs::Odometry odom;			//Odometry Message
datalogger::Altitude_datalogger alt;		//Custom Altitude Message

ofstream data;					//Container to write the data of the datalogger

ros::WallTime _time;				//Time of the system
ros::WallTime _actual_time;			//Actual time
ros::WallTime _last_time;			//Last time capture
double _time_data;				//Raw time
double _time_secs;				//Time in seconds
float _altitude;				//Altitude of the drone above the floor
float _last_altitude;				//Last altitude captured
float _vertical_vel;				//Vertical Velocity
float _heading;					//Heading of the drone
float _last_heading;				//Last Heading captured of the drone
float _heading_vel;				//Angular Velocity
float _output_altitude;				//Control signal Altitude
float _output_heading;				//Control signal Heading
double _orientationX;				//Quaternion in X axis
double _orientationY;				//Quaternion in Y axis
double _orientationZ;				//Quaternion in Z axis
double _orientationW;				//Quaternion in W axis
double roll,pitch,yaw;				//Roll, Pitch and Yaw of the drone


//Function to calculate the angular and vertical velocity
void get_vel()
{
	_vertical_vel = ((_altitude - _last_altitude) / _time_secs)*0.2;
	_heading_vel = ((_heading - _last_heading) / _time_secs)*0.2; 
}
//Function to calculate the current heading
void get_heading()
{
	tf::Quaternion q(_orientationX,_orientationY,_orientationZ,_orientationW);
	
	tf::Matrix3x3 m(q);
	m.getRPY(roll,pitch,yaw);
	
	//Convert it from radians into degrees
	roll = roll*(180/PI);
	pitch = pitch *(180/PI);
	yaw = yaw*(180/PI);
	
	_heading = yaw;
}

//Function to write the data into a file
void writeData()
{	_time = ros::WallTime::now();						//Get the current time
	_actual_time = _time;							//Save the current time
	_time_secs = _actual_time.toSec() - _last_time.toSec();			//Convert the time into seconds
	_time_data += _time_secs;						//Sum the seconds into the local container of the time
	get_heading();								//Get the heading
	get_vel();								//Get Angular and Vertical Velocity
	data.open("Bebop_data.dat", ios::out | ios::app);			//Look into the root folder for the output file
	data << _time_data << "," << _altitude << "," << _vertical_vel << "," << _heading << "," << _heading_vel << "," << _output_altitude << ","<< _output_heading << endl;							//Write the data into the file
	data.close();								//Release the file
	_last_time = _time;							//Save the current time into the last time container
	_last_altitude = _altitude;						//Save the current altitude into the last time container
	_last_heading = _heading; 						//Save the current heading into the last heading container
}

//Callback to obtain the odometry data
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	_orientationX = msg->pose.pose.orientation.x;
	_orientationY = msg->pose.pose.orientation.y;
	_orientationZ = msg->pose.pose.orientation.z;
	_orientationW = msg->pose.pose.orientation.w;
}
//Callback to obtain the altitude data
void altCallback(const datalogger::Altitude_datalogger::ConstPtr& msg)
{
	_altitude = msg->altitude;
}
//Callback to obtain the altitude PID data
void altPIDCallback(const std_msgs::Float64::ConstPtr& msg)
{
	_output_altitude = msg->data;
}
//Callback to obtain the heading PID data
void hdgPIDCallback(const std_msgs::Float64::ConstPtr& msg)
{
	_output_heading = msg->data;
}
//Main Program
int main(int argc, char** argv)
{
	ros::init(argc,argv,"bebop_datalogger");				//Advertise the node into the ROS enviroment
	ros::NodeHandle n;							//Create the node
 	ros::Subscriber bebopOdom;						//Subscribe to Odometry Data
	ros::Subscriber bebopAlt;						//Subscribe to Altitude Data
	ros::Subscriber bebopPIDalt;						//Subscribe to Altitude PID Data
	ros::Subscriber bebopPIDhdg;						//Subscribe to Heading PID Data
	
	//Initializing the subscribers
	bebopOdom = n.subscribe("/bebop/odom", 1000, odomCallback);
	bebopAlt = n.subscribe("/bebop/states/ardrone3/PilotingState/AltitudeChanged",1000,altCallback);
	bebopPIDalt = n.subscribe("/bebop/output_pid_alt",60,altPIDCallback);
	bebopPIDhdg = n.subscribe("/bebop/output_pid_hdg",60,hdgPIDCallback);

	//ros::Rate r(5);
	
	_time = ros::WallTime::now();						//Get the current time
	_last_time = _time;							//Save the current time into the last time container
	while(n.ok())								//Continue meanwhile <Ctrl + C> is not pressed 
	{
		ROS_INFO("\n %lf", _time_data);					//Display the current time
		ros::spinOnce();						//Refresh the topics
		writeData();							//Write the data into the file
		//r.sleep();			
		ros::Duration (0.2).sleep();					//Little delay to capture the data
	}
}
	

