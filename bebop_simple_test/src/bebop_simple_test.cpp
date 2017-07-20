#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>
#include <bebop_simple_test/PID_ROS.h>
#include <bebop_simple_test/Altitude.h>
#include <bebop_simple_test/ArcDrone.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <stdio.h>

#define PI 3.141592653589793238462

/*NUMBER OF TEST
	- 0: SIMPLE TAKE-OFF & LANDING
	- 1: SIMPLE ROUTINE
	- 2: COMPLEX ROUTINE
	- 3: CONTINUOUS ROTATING 
*/

//////////////////////////////
int test = 0; // <----- TEST /
//////////////////////////////

float _heading;
float _heading_t;
float _heading_t2;
float _arc;
float _arc2;
float _arc3;
float _fixed_arc;

//==============================Bebop_simple_test==========================
bool active = false;
bebop_simple_test::Altitude alt;

void altCallback(const bebop_simple_test::Altitude::ConstPtr& msg)
{	
	if (active)
	{
		float var1 = msg->altitude;
		alt.altitude = var1;
		printf("\n Altura %lf \n", var1);	
	}
}
void get_heading(float _orientationX, float _orientationY , float _orientationZ, float _orientationW)
{
	double roll;
	double pitch;
	double yaw;
	tf::Quaternion q(_orientationX,_orientationY,_orientationZ,_orientationW);
	
	tf::Matrix3x3 m(q);
	m.getRPY(roll,pitch,yaw);
	
	roll = roll*(180/PI);
	pitch = pitch *(180/PI);
	yaw = yaw*(180/PI);
	
	_heading = yaw;
}


void messageCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	float orX;
	float orY;
	float orZ;
	float orW;

	orX = msg->pose.pose.orientation.x;
	orY = msg->pose.pose.orientation.y;
	orZ = msg->pose.pose.orientation.z;
	orW = msg->pose.pose.orientation.w;
	
	get_heading(orX,orY,orZ,orW);
	printf("\n Heading %lf \n", _heading);
}

	
int main(int argc, char** argv)
{			
	//PID
        PID pid_alt;
	pid_alt.gpro = 3.5;
	pid_alt.ginteg = 0.0;
        pid_alt.gder = 0.0001;

	PID_h pid_hdg;
	pid_hdg.gpro = 0.15;
	pid_hdg.ginteg = 0.0;
	pid_hdg.gder = 0.00002;
	
	ros::init(argc,argv,"bebop_test");
	ros::NodeHandle n;

	ros::Publisher takeoff_bebop;
	ros::Publisher land_bebop;
	ros::Publisher cmd_vel_pub_bebop;
	ros::Subscriber bebopOdom;
	ros::Publisher correct_hdg;
	ros::Subscriber bebopAlt;
	ros::Publisher correct_alt;

	std_msgs::Empty msg_takeoff, msg_land;
	geometry_msgs::Twist cmd_vel_bebop;
	std_msgs::Float64 output_alt_pid;
	std_msgs::Float64 output_hdg_pid;

	takeoff_bebop = n.advertise<std_msgs::Empty>("/bebop/takeoff",1000);
	land_bebop = n.advertise<std_msgs::Empty>("/bebop/land",1000);
	cmd_vel_pub_bebop = n.advertise<geometry_msgs::Twist>("/bebop/cmd_vel",1000);
	bebopOdom = n.subscribe("/bebop/odom",1000,messageCallback);
	correct_hdg = n.advertise<std_msgs::Float64>("/bebop/output_pid_hdg",30);
	bebopAlt = n.subscribe("/bebop/states/ardrone3/PilotingState/AltitudeChanged",1000,altCallback);
	correct_alt = n.advertise<std_msgs::Float64>("/bebop/output_pid_alt",30);

	
	cmd_vel_bebop.linear.x = 0;
	cmd_vel_bebop.linear.y = 0;
	cmd_vel_bebop.linear.z = 0;

	cmd_vel_bebop.angular.x = 0;	
	cmd_vel_bebop.angular.y = 0;	
	cmd_vel_bebop.angular.z = 0;
	
	
	//ros::Rate r(100);
	
	if(test == 0)
	{
		ros::Duration(5).sleep();
		printf("\n========== T A K E O F F ==========\n");
		takeoff_bebop.publish(msg_takeoff);
		ros::Duration(4.5).sleep();
		printf("\n========== L A N D ==========\n");
		land_bebop.publish(msg_land);
		return 0;
	}
	else
	{
		ros::Duration(5).sleep();
		printf("\n========== T A K E O F F ==========\n");
		takeoff_bebop.publish(msg_takeoff);
		ros::Duration(4.5).sleep();

		cmd_vel_pub_bebop.publish(cmd_vel_bebop);
		ros::Duration(0.5).sleep();
	
		active = true;
		ros::spinOnce();
		if(test == 1)
		{
			//SIMPLE ROUTINE

			//HOVER
			printf("\n======== H O V E R ==========\n");
			cmd_vel_bebop.linear.x = 0;
			cmd_vel_bebop.linear.y = 0;
			cmd_vel_bebop.linear.z = 0;

			cmd_vel_bebop.angular.x = 0;	
			cmd_vel_bebop.angular.y = 0;	
			cmd_vel_bebop.angular.z = 0;

			cmd_vel_pub_bebop.publish(cmd_vel_bebop);
			ros::spinOnce();
        		ros::spinOnce();
			ros::Duration(5).sleep();
			ros::spinOnce();

			printf("\n========== D E S C E N D I N G ==========\n");	
			cmd_vel_bebop.linear.x = 0;
			cmd_vel_bebop.linear.y = 0;
			//cmd_vel_bebop.linear.z = 0.27;

			cmd_vel_bebop.angular.x = 0;	
			cmd_vel_bebop.angular.y = 0;	
			cmd_vel_bebop.angular.z = 0;	

			for(int i=0;i<720;i++)//60
			{
				cmd_vel_bebop.linear.z = pid_alt.mis(alt.altitude,0.60);
				output_alt_pid.data = cmd_vel_bebop.linear.z;
				//printf("<<%f>>\n",cmd_vel_bebop.linear.z);
				cmd_vel_pub_bebop.publish(cmd_vel_bebop);
				correct_alt.publish(output_alt_pid);
				ros::spinOnce();
				ros::Duration(0.016).sleep();
			}	

			//HOVER
			printf("\n======== H O V E R ==========\n");
			cmd_vel_bebop.linear.x = 0;
			cmd_vel_bebop.linear.y = 0;
			cmd_vel_bebop.linear.z = 0;

			cmd_vel_bebop.angular.x = 0;	
			cmd_vel_bebop.angular.y = 0;	
			cmd_vel_bebop.angular.z = 0;

			cmd_vel_pub_bebop.publish(cmd_vel_bebop);
			ros::spinOnce();
        		ros::spinOnce();
			ros::Duration(5).sleep();
			ros::spinOnce();
	
			printf("\n======== R O T A T I N G [90º] ==========\n");	

			cmd_vel_bebop.linear.x = 0;
			cmd_vel_bebop.linear.y = 0;
			cmd_vel_bebop.linear.z = 0;

			cmd_vel_bebop.angular.x = 0;	
			cmd_vel_bebop.angular.y = 0;	
			cmd_vel_bebop.angular.z = 0;
	
			_heading_t = constrainAngle(_heading);
			_arc = calcArc(90,_heading_t); 
			//_fixed_heading = _heading;
	
			for(int i=0;i<800;i++)//60
			{
				_heading_t = constrainAngle(_heading);
	        		_arc2 = calcArc(90,_heading_t);
				_arc3 = _arc - _arc2;
				cmd_vel_bebop.angular.z = pid_hdg.mis(_arc3,_arc);
				output_hdg_pid.data = cmd_vel_bebop.angular.z;
				/*printf(" <<%f>>\n",cmd_vel_bebop.angular.z);
				printf(" $$%f$$\n",_arc);
				printf(" !!%f!!\n",_arc2);*/
				cmd_vel_pub_bebop.publish(cmd_vel_bebop);
				correct_hdg.publish(output_hdg_pid);
				ros::spinOnce();
				ros::Duration(0.016).sleep();
			}
	
			//HOVER
			printf("\n======== H O V E R ==========\n");
			cmd_vel_bebop.linear.x = 0;
			cmd_vel_bebop.linear.y = 0;
			cmd_vel_bebop.linear.z = 0;

			cmd_vel_bebop.angular.x = 0;	
			cmd_vel_bebop.angular.y = 0;	
			cmd_vel_bebop.angular.z = 0;

			cmd_vel_pub_bebop.publish(cmd_vel_bebop);
			ros::spinOnce();
        		ros::spinOnce();
			ros::Duration(5).sleep();
			ros::spinOnce();

			printf("\n========== E L E V A T I N G ==========\n");	
			cmd_vel_bebop.linear.x = 0;
			cmd_vel_bebop.linear.y = 0;
			//cmd_vel_bebop.linear.z = 0.27;

			cmd_vel_bebop.angular.x = 0;	
			cmd_vel_bebop.angular.y = 0;	
			cmd_vel_bebop.angular.z = 0;	

			for(int i=0;i<720;i++)//60
			{
				cmd_vel_bebop.linear.z = pid_alt.mis(alt.altitude,1.10);
				output_alt_pid.data = cmd_vel_bebop.linear.z;
				//printf("<<%f>>\n",cmd_vel_bebop.linear.z);
				cmd_vel_pub_bebop.publish(cmd_vel_bebop);
				correct_alt.publish(output_alt_pid);
				ros::spinOnce();
				ros::Duration(0.016).sleep();
			}
	
			//HOVER
			printf("\n======== H O V E R ==========\n");
			cmd_vel_bebop.linear.x = 0;
			cmd_vel_bebop.linear.y = 0;
			cmd_vel_bebop.linear.z = 0;

			cmd_vel_bebop.angular.x = 0;	
			cmd_vel_bebop.angular.y = 0;	
			cmd_vel_bebop.angular.z = 0;

			cmd_vel_pub_bebop.publish(cmd_vel_bebop);
			ros::spinOnce();
        		ros::spinOnce();
			ros::Duration(5).sleep();
			ros::spinOnce();

			printf("\n======== R O T A T I N G [0º] ==========\n");	

			cmd_vel_bebop.linear.x = 0;
			cmd_vel_bebop.linear.y = 0;
			cmd_vel_bebop.linear.z = 0;

			cmd_vel_bebop.angular.x = 0;	
			cmd_vel_bebop.angular.y = 0;	
			cmd_vel_bebop.angular.z = 0;

			_heading_t = constrainAngle(_heading);
			_arc = calcArc(0,_heading_t); 
			//_fixed_heading = _heading;
	
			for(int i=0;i<800;i++)//60
			{
				_heading_t = constrainAngle(_heading);
	        		_arc2 = calcArc(0,_heading_t);
				_arc3 = _arc - _arc2;
				cmd_vel_bebop.angular.z = pid_hdg.mis(_arc3,_arc);
				output_hdg_pid.data = cmd_vel_bebop.angular.z;
				/*printf(" <<%f>>\n",cmd_vel_bebop.angular.z);
				printf(" $$%f$$\n",_arc);
				printf(" !!%f!!\n",_arc2);*/
				cmd_vel_pub_bebop.publish(cmd_vel_bebop);
				correct_hdg.publish(output_hdg_pid);
				ros::spinOnce();
				ros::Duration(0.016).sleep();
			}
	
			//HOVER
			printf("\n======== H O V E R ==========\n");
			cmd_vel_bebop.linear.x = 0;
			cmd_vel_bebop.linear.y = 0;
			cmd_vel_bebop.linear.z = 0;

			cmd_vel_bebop.angular.x = 0;	
			cmd_vel_bebop.angular.y = 0;	
			cmd_vel_bebop.angular.z = 0;

			cmd_vel_pub_bebop.publish(cmd_vel_bebop);
			ros::spinOnce();
        		ros::spinOnce();
			ros::Duration(5).sleep();
			ros::spinOnce();	

			active = false;

		}
		if(test == 2)
		{
			//COMPLEX ROUTINE

			//HOVER
			printf("\n======== H O V E R ==========\n");
			cmd_vel_bebop.linear.x = 0;
			cmd_vel_bebop.linear.y = 0;
			cmd_vel_bebop.linear.z = 0;

			cmd_vel_bebop.angular.x = 0;	
			cmd_vel_bebop.angular.y = 0;	
			cmd_vel_bebop.angular.z = 0;

			cmd_vel_pub_bebop.publish(cmd_vel_bebop);
			ros::spinOnce();
        		ros::spinOnce();
			ros::Duration(5).sleep();
			ros::spinOnce();

			printf("\n========== D E S C E N D I N G  &  90 º ==========\n");	
			cmd_vel_bebop.linear.x = 0;
			cmd_vel_bebop.linear.y = 0;
			cmd_vel_bebop.linear.z = 0;

			cmd_vel_bebop.angular.x = 0;	
			cmd_vel_bebop.angular.y = 0;	
			cmd_vel_bebop.angular.z = 0;

			_heading_t = constrainAngle(_heading);
			_arc = calcArc(90,_heading_t);	

			for(int i=0;i<720;i++)//60
			{
				_heading_t = constrainAngle(_heading);
	        		_arc2 = calcArc(90,_heading_t);
				_arc3 = _arc - _arc2;
				cmd_vel_bebop.angular.z = pid_hdg.mis(_arc3,_arc);
				output_hdg_pid.data = cmd_vel_bebop.angular.z;
				cmd_vel_bebop.linear.z = pid_alt.mis(alt.altitude,0.60);
				output_alt_pid.data = cmd_vel_bebop.linear.z;
				//printf("<<%f>>\n",cmd_vel_bebop.linear.z);
				cmd_vel_pub_bebop.publish(cmd_vel_bebop);
				correct_alt.publish(output_alt_pid);
				correct_hdg.publish(output_hdg_pid);
				ros::spinOnce();
				ros::Duration(0.016).sleep();
			}
	
	
			printf("\n========== E L E V A T I N G  &  0 º ==========\n");	
			cmd_vel_bebop.linear.x = 0;
			cmd_vel_bebop.linear.y = 0;
			cmd_vel_bebop.linear.z = 0;

			cmd_vel_bebop.angular.x = 0;	
			cmd_vel_bebop.angular.y = 0;	
			cmd_vel_bebop.angular.z = 0;
	
			_heading_t = constrainAngle(_heading);
			_arc = calcArc(0,_heading_t);	

			for(int i=0;i<720;i++)//60
			{
				_heading_t = constrainAngle(_heading);
	        		_arc2 = calcArc(0,_heading_t);
				_arc3 = _arc - _arc2;
				cmd_vel_bebop.angular.z = pid_hdg.mis(_arc3,_arc);
				output_hdg_pid.data = cmd_vel_bebop.angular.z;
				cmd_vel_bebop.linear.z = pid_alt.mis(alt.altitude,1.10);
				output_alt_pid.data = cmd_vel_bebop.linear.z;
				//printf("<<%f>>\n",cmd_vel_bebop.linear.z);
				cmd_vel_pub_bebop.publish(cmd_vel_bebop);
				correct_alt.publish(output_alt_pid);
				correct_hdg.publish(output_hdg_pid);
				ros::spinOnce();
				ros::Duration(0.016).sleep();
			}
		}
		if(test == 3)
		{
			//CONTINUOUS ROTATING 	
			
			//HOVER
			printf("\n======== H O V E R ==========\n");
			cmd_vel_bebop.linear.x = 0;
			cmd_vel_bebop.linear.y = 0;
			cmd_vel_bebop.linear.z = 0;

			cmd_vel_bebop.angular.x = 0;	
			cmd_vel_bebop.angular.y = 0;	
			cmd_vel_bebop.angular.z = 0;

			cmd_vel_pub_bebop.publish(cmd_vel_bebop);
			ros::spinOnce();
        		ros::spinOnce();
			ros::Duration(5).sleep();
			ros::spinOnce();

			printf("\n========== R O T A T I N G ==========\n");	
			cmd_vel_bebop.linear.x = 0;
			cmd_vel_bebop.linear.y = 0;
			cmd_vel_bebop.linear.z = 0;

			cmd_vel_bebop.angular.x = 0;	
			cmd_vel_bebop.angular.y = 0;	
			cmd_vel_bebop.angular.z = 0;

			_heading_t = constrainAngle(_heading);
			_arc = calcArc(_heading_t + 10,_heading_t);	

			while(n.ok())
			{
				_heading_t = constrainAngle(_heading);
	        		_arc2 = calcArc(_heading_t + 30,_heading_t);
				_arc3 = _arc - _arc2;
				cmd_vel_bebop.angular.z = pid_hdg.mis(_arc3,_arc);
				output_hdg_pid.data = cmd_vel_bebop.angular.z;
				cmd_vel_bebop.linear.z = pid_alt.mis(alt.altitude,1.20);
				output_alt_pid.data = cmd_vel_bebop.linear.z;
				//printf("<<%f>>\n",cmd_vel_bebop.linear.z);
				cmd_vel_pub_bebop.publish(cmd_vel_bebop);
				correct_alt.publish(output_alt_pid);
				correct_hdg.publish(output_hdg_pid);
				ros::spinOnce();
				ros::Duration(1).sleep();
			}
		}
	}	


	printf("\n========== L A N D ==========\n");
	land_bebop.publish(msg_land);
	ros::spinOnce();
	//r.sleep();
}


