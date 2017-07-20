//Coded by Alexis Guijarro
//"Angle To Center" segment by Salvador Figuerola 

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Point.h>

#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>

#include <vicon_test/PID_ROS.h>
#include <vicon_test/ArcDrone.h>

#include <stdio.h>
#include <cmath>
#include <math.h>

#define PI 3.141592653589793238462

//Global Variables
	
double heading;
int btn_emergency;
geometry_msgs::Twist cmd_vel_bebop;
ros::WallTime _time;
ros::WallTime _actual_time;
ros::WallTime _last_time;
double _time_data;
double _time_secs;
double _time_wait;


///////////////// V I C O N - B E B O P \\\\\\\\\\\\\\\\\\\\\\\\\\\\

struct drone
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
}bebop;

struct Point
{
	double x;
	double y;
};

void getJoyState(const sensor_msgs::Joy::ConstPtr& js)
{
	btn_emergency = js->buttons[0];
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

	bebop.abs_x = bebop._posX;
	bebop.abs_y = bebop._posY;
}
void getAngtoCenter(void)
{
	if(bebop.abs_x < 0)
	{
	bebop.abs_x = bebop.abs_x * -1;
	}
	if(bebop.abs_y < 0)
	{
	bebop.abs_y = bebop.abs_y * -1;
	}

	bebop.angle_res = atan2(bebop._posY,bebop._posX);
	bebop.angle_res = bebop.angle_res * (180/PI);

	bebop.angle_res = constrainAngle(bebop.angle_res);

	if(bebop._posX > 0 and bebop._posY > 0)
	{
		bebop.angle_arc = bebop.angle_res +180;
	}
	else if(bebop._posX < 0 and bebop._posY > 0)
	{
		bebop.angle_arc = bebop.angle_res +180;
	}
	else if(bebop._posX < 0 and bebop._posY < 0)
	{
		bebop.angle_arc = bebop.angle_res -180;
	}
	else if(bebop._posX > 0 and bebop._posY < 0)
	{
		bebop.angle_arc = bebop.angle_res -180;
	}
}

//Deadzone Circle
bool insideCircle()
{	
	bool inside;
	double radio = sqrt(pow(bebop._posX, 2.0) + pow(bebop._posY, 2.0));
	if(radio < 0.50)
	{
		inside = true;
	} 
	else 
	{
		inside = false;
	}
	return inside;

}
int main(int argc, char** argv)
{
	ros::init(argc,argv,"bebop_vicon");
	ros::NodeHandle n;
	ros::Subscriber joy_sub,vicon_sub;
	ros::Publisher takeoff_bebop;
	ros::Publisher land_bebop;
	ros::Publisher cmd_vel_pub_bebop;
	joy_sub = n.subscribe("/joy",1000,getJoyState);
	vicon_sub = n.subscribe("/vicon/Drone_LS/Drone_LS",1000,getBebopPos);
	cmd_vel_pub_bebop = n.advertise<geometry_msgs::Twist>("/bebop/cmd_vel",1000);
	takeoff_bebop = n.advertise<std_msgs::Empty>("/bebop/takeoff",1000);
	land_bebop = n.advertise<std_msgs::Empty>("/bebop/land",1000);

	PID_h pid_hdg,pid_pos,pid_alt;
	pid_hdg.gpro = 0.10;
	pid_hdg.ginteg = 0.0;
	pid_hdg.gder = 0.00002;
	
	pid_pos.gpro = 1.353; // 1.3525
	pid_pos.ginteg = 0.0;
	pid_pos.gder = 0.0035;
	
	pid_alt.gpro = 3.5;
	pid_alt.ginteg = 0.0;
        pid_alt.gder = 0.0001;

	std_msgs::Empty msg_takeoff, msg_land;

	//Waypoints
	
	Point p[5];
	p[0].x = 0;
	p[0].y = 0;
	p[1].x = 0.83;
	p[1].y = -0.80;
	p[2].x = 0.85;
	p[2].y = 0.90;
	p[3].x = -0.60;
	p[3].y = 0.85;
	p[4].x = -0.60;
	p[4].y = -0.85;


	double _arc,_arc2,_arc3;
	
	//Rotation Control
	bool rot_control = true;

	cmd_vel_bebop.linear.x = 0;
	cmd_vel_bebop.linear.y = 0;
	cmd_vel_bebop.linear.z = 0;

	cmd_vel_bebop.angular.x = 0;	
	cmd_vel_bebop.angular.y = 0;	
	cmd_vel_bebop.angular.z = 0;

	ros::spinOnce();	
	
	double hdg_target = 90;
	bool correct_hdg = false;
	bool correcting_hdg = false;
	bool inside_circle;
	
	
	heading = constrainAngle(heading);
	_arc = calcArc(hdg_target,heading); 
	
	//Time necesary to setup the publishers	
	ros::Duration(2).sleep();	

	ros::Rate r(100);

	printf("\n========== T A K E O F F ==========\n");
	takeoff_bebop.publish(msg_takeoff);
	ros::Duration(4.5).sleep();
	
	/*printf("\n========== L A N D ==========\n");
	land_bebop.publish(msg_land);*/
	

	cmd_vel_pub_bebop.publish(cmd_vel_bebop);
	ros::Duration(0.5).sleep();
	
	int waypoint = 0;

	while(n.ok())
	{
		if(btn_emergency)
		{
			ros::Duration(0.525).sleep();
			printf("\n========== L A N D [ J S ]==========\n");
			land_bebop.publish(msg_land);
			break;
		}

		inside_circle = insideCircle();
		getAngtoCenter();
		heading = constrainAngle(heading);
		_arc = calcArc(bebop.angle_arc,heading);
		ros::Duration(0.0001).sleep();
		_arc2 = calcArc(bebop.angle_arc,heading);
		_arc3 = _arc - _arc2;

		if(!inside_circle)
		{
			cmd_vel_bebop.angular.z = pid_hdg.mis(_arc3,_arc);
		}

		if(bebop._posX < (p[waypoint].x + 0.15)  && bebop._posX > (p[waypoint].x - 0.15) && bebop._posY < (p[waypoint].y + 0.15) && bebop._posY > (p[waypoint].y - 0.15) && _arc3 < (_arc + 10) && _arc3 > (_arc - 10 ))
		{
			_time = ros::WallTime::now();
			_actual_time = _time;
			_time_secs = _actual_time.toSec() - _last_time.toSec();
			_time_data += _time_secs;
			if(_time_data > 2)
			{
				/*cmd_vel_bebop.linear.x = 0;
				cmd_vel_bebop.linear.y = 0;
				cmd_vel_bebop.linear.z = 0;

				cmd_vel_bebop.angular.x = 0;	
				cmd_vel_bebop.angular.y = 0;	
				cmd_vel_bebop.angular.z = 0;*/
	
				cmd_vel_pub_bebop.publish(cmd_vel_bebop);
		
				printf("\n ZONA [%d] ALCANZADA \n",waypoint);
				ros::Duration(5).sleep();		
				_time_data = 0;
				waypoint = waypoint++;
				if (waypoint > 4)
				{
					waypoint = 0;
				}	
			}		
		}
		else
		{
			if (rot_control)
			{
				bebop._cmdX = pid_pos.mis(bebop._posX,p[waypoint].x);
				bebop._cmdY = pid_pos.mis(bebop._posY,p[waypoint].y);
				bebop.rot_cmd_x = bebop._cmdX*cos(bebop._yawRAD) + bebop._cmdY*sin(bebop._yawRAD);
				bebop.rot_cmd_y = bebop._cmdX*sin(bebop._yawRAD) - bebop._cmdY*cos(bebop._yawRAD);
					
					
				cmd_vel_bebop.linear.x = bebop.rot_cmd_x;
				cmd_vel_bebop.linear.y = -bebop.rot_cmd_y;

			}
			else
			{
				cmd_vel_bebop.linear.x = pid_pos.mis(bebop._posX,p[waypoint].x);
				cmd_vel_bebop.linear.y = pid_pos.mis(bebop._posY,p[waypoint].y);
			}
				_time_data = 0;
				_time_wait = 0;
		}	
			correct_hdg = false;
		
		cmd_vel_bebop.linear.z = pid_alt.mis(bebop._posZ,1.10);
		cmd_vel_pub_bebop.publish(cmd_vel_bebop);
		//ROS_INFO("Altura: %lf  Heading: %lf",bebop._posZ,bebop._yaw);
		ROS_INFO("X: %lf  Y: %lf CMDX: %lf  CMDY: %lf",bebop._posX,bebop._posY, cmd_vel_bebop.linear.x,cmd_vel_bebop.linear.y);
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
	return 0;
}



