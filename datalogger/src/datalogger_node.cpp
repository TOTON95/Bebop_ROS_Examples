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

#define PI 3.141592653589793238462 //PI

using namespace std;

nav_msgs::Odometry odom;
datalogger::Altitude_datalogger alt;

ofstream data;

ros::WallTime _time;
ros::WallTime _actual_time;
ros::WallTime _last_time;
double _time_data;
double _time_secs;
float _altitude;
float _last_altitude;
float _vertical_vel;
float _heading;
float _last_heading;
float _heading_vel;
float _output_altitude;
float _output_heading;
double _orientationX;
double _orientationY;
double _orientationZ;
double _orientationW;
double roll,pitch,yaw;



void get_vel()
{
	_vertical_vel = ((_altitude - _last_altitude) / _time_secs)*0.2;
	_heading_vel = ((_heading - _last_heading) / _time_secs)*0.2; 
}

void get_heading()
{
	tf::Quaternion q(_orientationX,_orientationY,_orientationZ,_orientationW);
	
	tf::Matrix3x3 m(q);
	m.getRPY(roll,pitch,yaw);
	
	roll = roll*(180/PI);
	pitch = pitch *(180/PI);
	yaw = yaw*(180/PI);
	
	_heading = yaw;
}

void writeData()
{	_time = ros::WallTime::now();
	_actual_time = _time;
	_time_secs = _actual_time.toSec() - _last_time.toSec();
	_time_data += _time_secs;
	get_heading();
	get_vel();
	data.open("Bebop_data.dat", ios::out | ios::app);
	data << _time_data << "," << _altitude << "," << _vertical_vel << "," << _heading << "," << _heading_vel << "," << _output_altitude << ","<< _output_heading << endl;
	data.close();
	_last_time = _time;
	_last_altitude = _altitude;
	_last_heading = _heading; 
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	_orientationX = msg->pose.pose.orientation.x;
	_orientationY = msg->pose.pose.orientation.y;
	_orientationZ = msg->pose.pose.orientation.z;
	_orientationW = msg->pose.pose.orientation.w;
}

void altCallback(const datalogger::Altitude_datalogger::ConstPtr& msg)
{
	_altitude = msg->altitude;
}

void altPIDCallback(const std_msgs::Float64::ConstPtr& msg)
{
	_output_altitude = msg->data;
}

void hdgPIDCallback(const std_msgs::Float64::ConstPtr& msg)
{
	_output_heading = msg->data;
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"bebop_datalogger");
	ros::NodeHandle n;
 	ros::Subscriber bebopOdom;
	ros::Subscriber bebopAlt;
	ros::Subscriber bebopPIDalt;
	ros::Subscriber bebopPIDhdg;
	
	bebopOdom = n.subscribe("/bebop/odom", 1000, odomCallback);
	bebopAlt = n.subscribe("/bebop/states/ardrone3/PilotingState/AltitudeChanged",1000,altCallback);
	bebopPIDalt = n.subscribe("/bebop/output_pid_alt",60,altPIDCallback);
	bebopPIDhdg = n.subscribe("/bebop/output_pid_hdg",60,hdgPIDCallback);

	//ros::Rate r(5);
	
	_time = ros::WallTime::now();
	_last_time = _time;
	while(n.ok())
	{
		ROS_INFO("\n %lf", _time_data);
		ros::spinOnce();
		writeData();
		//r.sleep();
		ros::Duration (0.2).sleep();
	}
}
	

