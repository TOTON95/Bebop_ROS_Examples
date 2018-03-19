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
int test = 2; // <----- TEST /
//////////////////////////////

//Heading Variables
float _heading;
float _heading_t;
float _heading_t2;
float _arc;
float _arc2;
float _arc3;
float _fixed_arc;

//==============================Bebop_simple_test==========================

bool active = false;			//Store the actual state, true (the drone is active and flying)
bebop_simple_test::Altitude alt;	//Custom ROS Variable to store the altitude 

//Callback used to adquire the data from the ultrasonic sensor under the drone
void altCallback(const bebop_simple_test::Altitude::ConstPtr& msg)
{	
	if (active)
	{
		float var1 = msg->altitude; 		//Altitude field expects a float variable
		alt.altitude = var1;
		printf("\n Altura %lf \n", var1);	//Altitude in meters
	}
}
//Function used to convert the heading data onboard from the drone. 
void get_heading(float _orientationX, float _orientationY , float _orientationZ, float _orientationW)
{
	double roll;
	double pitch;
	double yaw;
	tf::Quaternion q(_orientationX,_orientationY,_orientationZ,_orientationW);
	
	tf::Matrix3x3 m(q);
	m.getRPY(roll,pitch,yaw);
	
	//Roll, Pitch and Yaw are in radians, must convert it to degrees
	roll = roll*(180/PI);
	pitch = pitch *(180/PI);
	yaw = yaw*(180/PI);
	
	//Yaw value copied into the global variable _heading for further use 
	_heading = yaw;
}

//Callback used to get the heading data onboard from the drone. 
void messageCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	float orX;
	float orY;
	float orZ;
	float orW;
	
	//Getting the data from Odometry variable and converting it
	orX = msg->pose.pose.orientation.x;
	orY = msg->pose.pose.orientation.y;
	orZ = msg->pose.pose.orientation.z;
	orW = msg->pose.pose.orientation.w;
	
	get_heading(orX,orY,orZ,orW);	
	printf("\n Heading %lf \n", _heading);		//In degrees
}


//Main function 
int main(int argc, char** argv)
{			
	//PID
        PID pid_alt;					//PID Altitude 
	pid_alt.gpro = 3.5;				//Proportional Constant
	pid_alt.ginteg = 0.0;				//Integral Constant
        pid_alt.gder = 0.0001;				//Derivative Constant

	PID_h pid_hdg;					//PID Heading
	pid_hdg.gpro = 0.15;				//Proportional Constant
	pid_hdg.ginteg = 0.0;				//Integral Constant
	pid_hdg.gder = 0.00002;				//Derivative Constant
	
	ros::init(argc,argv,"bebop_test");		//Initializing a ROS program
	ros::NodeHandle n;				//Creating a ROS node

	ros::Publisher takeoff_bebop;			//Sends the message to make the drone to take off 
	ros::Publisher land_bebop;			//Sends the message to make the drone to land
	ros::Publisher cmd_vel_pub_bebop;		//Sends the message to move the drone
	ros::Subscriber bebopOdom;			//Receives the data from the sensors onboard
	ros::Publisher correct_hdg;			//Send the signal to correct the heading
	ros::Subscriber bebopAlt;			//Receives the alttitude from the built-in sensor
	ros::Publisher correct_alt;			//Send the signal to correct the alttitude 

	std_msgs::Empty msg_takeoff, msg_land;		//Messages created to take-off and land procedures 
	geometry_msgs::Twist cmd_vel_bebop;		//Contains the data to make the drone fly in certain direction
	std_msgs::Float64 output_alt_pid;		//The signal of the PID to control over the alttitude
	std_msgs::Float64 output_hdg_pid;		//The signal of the PID to control over the heading

	takeoff_bebop = n.advertise<std_msgs::Empty>("/bebop/takeoff",1000);					//Publish data to the take-off topic
	land_bebop = n.advertise<std_msgs::Empty>("/bebop/land",1000);						//Publish data to the land topic
	cmd_vel_pub_bebop = n.advertise<geometry_msgs::Twist>("/bebop/cmd_vel",1000);				//Publish data to the movement topic
	bebopOdom = n.subscribe("/bebop/odom",1000,messageCallback);						//Receives data from the sensors
	correct_hdg = n.advertise<std_msgs::Float64>("/bebop/output_pid_hdg",30);				//Publish data of the hdg pid signal
	bebopAlt = n.subscribe("/bebop/states/ardrone3/PilotingState/AltitudeChanged",1000,altCallback);	//Receives data of the alttitude
	correct_alt = n.advertise<std_msgs::Float64>("/bebop/output_pid_alt",30);				//Publish data of the alt pid signal

	//The data from bebopAlt and alttitude from bebopOdom are not the same. Refer to bebopAlt to measure the alttitude
	
	
	//Puts the drone in HOVER setting everything to 0.
	cmd_vel_bebop.linear.x = 0;
	cmd_vel_bebop.linear.y = 0;
	cmd_vel_bebop.linear.z = 0;

	cmd_vel_bebop.angular.x = 0;	
	cmd_vel_bebop.angular.y = 0;	
	cmd_vel_bebop.angular.z = 0;
	
	
	//ros::Rate r(100);e 
	
	//SIMPLE TAKE-OFF & LANDING
	//Its highly recommended to perform this test first to check if everything its fine with ROS and the drone itself
	if(test == 0)
	{
		ros::Duration(5).sleep();					//Waits 5 seconds
		printf("\n========== T A K E O F F ==========\n");		
		takeoff_bebop.publish(msg_takeoff);				//Give the order to take-off
		ros::Duration(4.5).sleep();					//Waits 4.5 seconds
		printf("\n========== L A N D ==========\n");
		land_bebop.publish(msg_land);					//Give the order to land 
		return 0;
	}
	else
	{
		ros::Duration(5).sleep();					//Waits 5 seconds
		printf("\n========== T A K E O F F ==========\n");		
		takeoff_bebop.publish(msg_takeoff);				//Give the order to take-off
		ros::Duration(4.5).sleep();					//Waits 4.5 seconds
					
		cmd_vel_pub_bebop.publish(cmd_vel_bebop);			//Sets the drone to HOVER
		ros::Duration(0.5).sleep();					//Waits 0.5 seconds to accomplish the order
	
		active = true;							//Indicates if the drone is active
		ros::spinOnce();						//Refresh the data from the topics
		if(test == 1)
		{
			//SIMPLE ROUTINE

			//HOVER
			printf("\n======== H O V E R ==========\n");
			cmd_vel_bebop.linear.x = 0;				//Puts the drone in HOVER setting everything to 0.
			cmd_vel_bebop.linear.y = 0;
			cmd_vel_bebop.linear.z = 0;

			cmd_vel_bebop.angular.x = 0;	
			cmd_vel_bebop.angular.y = 0;	
			cmd_vel_bebop.angular.z = 0;

			cmd_vel_pub_bebop.publish(cmd_vel_bebop);		//Load the order to hover
			ros::spinOnce();					//Refresh the topics
        		ros::spinOnce();
			ros::Duration(5).sleep();				//Stays on hover 5 seconds
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
				cmd_vel_bebop.linear.z = pid_alt.mis(alt.altitude,0.60); 	//Sets the alttitude to 60 centimeters above the floor
				output_alt_pid.data = cmd_vel_bebop.linear.z;			//Send the data to the altittude slot of the message
				//printf("<<%f>>\n",cmd_vel_bebop.linear.z);
				cmd_vel_pub_bebop.publish(cmd_vel_bebop);			//Delivers the data to move the drone
				correct_alt.publish(output_alt_pid);				//Publish the data to record it using a subscriber 
				ros::spinOnce();						//Refresh the data from the topics
				ros::Duration(0.016).sleep();					//Little delay to avoid a  too quick action
			}	

			//HOVER
			printf("\n======== H O V E R ==========\n");
			cmd_vel_bebop.linear.x = 0;						//Puts the drone in HOVER setting everything to 0.
			cmd_vel_bebop.linear.y = 0;
			cmd_vel_bebop.linear.z = 0;

			cmd_vel_bebop.angular.x = 0;	
			cmd_vel_bebop.angular.y = 0;	
			cmd_vel_bebop.angular.z = 0;

			cmd_vel_pub_bebop.publish(cmd_vel_bebop);				//Load the order to hover
			ros::spinOnce();							//Refresh the topics
        		ros::spinOnce();
			ros::Duration(5).sleep();						//Stays on hover 5 seconds
			ros::spinOnce();
	
			printf("\n======== R O T A T I N G [90º] ==========\n");	

			cmd_vel_bebop.linear.x = 0;
			cmd_vel_bebop.linear.y = 0;
			cmd_vel_bebop.linear.z = 0;

			cmd_vel_bebop.angular.x = 0;	
			cmd_vel_bebop.angular.y = 0;	
			cmd_vel_bebop.angular.z = 0;
	
			_heading_t = constrainAngle(_heading);					//Reads the current heading
			_arc = calcArc(90,_heading_t); 						//Calculates the arc to a fixed orientation (90º)
			//_fixed_heading = _heading;			
	
			for(int i=0;i<800;i++)//60
			{
				_heading_t = constrainAngle(_heading);				//Reads the current heading
	        		_arc2 = calcArc(90,_heading_t);					//Calculates the arc to a fixed orientation (90º)
				_arc3 = _arc - _arc2;						//Calculates the difference between the target arc and the current one.
				cmd_vel_bebop.angular.z = pid_hdg.mis(_arc3,_arc);		//Data is sent to the PID control
				output_hdg_pid.data = cmd_vel_bebop.angular.z;			//Stream the data to the control heading topic
				/*printf(" <<%f>>\n",cmd_vel_bebop.angular.z);
				printf(" $$%f$$\n",_arc);
				printf(" !!%f!!\n",_arc2);*/
				cmd_vel_pub_bebop.publish(cmd_vel_bebop);			//Load the order to move
				correct_hdg.publish(output_hdg_pid);				//Publish the order
				ros::spinOnce();						//Refresh the topics
				ros::Duration(0.016).sleep();					//Little delay to accomplish the order
			}
	
			//HOVER
			printf("\n======== H O V E R ==========\n");
			cmd_vel_bebop.linear.x = 0;						//Puts the drone in HOVER setting everything to 0.
			cmd_vel_bebop.linear.y = 0;
			cmd_vel_bebop.linear.z = 0;

			cmd_vel_bebop.angular.x = 0;	
			cmd_vel_bebop.angular.y = 0;	
			cmd_vel_bebop.angular.z = 0;

			cmd_vel_pub_bebop.publish(cmd_vel_bebop);				//Load the order to hover
			ros::spinOnce();							//Refresh the topics
        		ros::spinOnce();
			ros::Duration(5).sleep();						//Stays on hover 5 seconds
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
				cmd_vel_bebop.linear.z = pid_alt.mis(alt.altitude,1.10);	//Reads the current altittude and Data is sent to the PID control (1.10 mts)
				output_alt_pid.data = cmd_vel_bebop.linear.z;			//Stream the altittude signal
				//printf("<<%f>>\n",cmd_vel_bebop.linear.z);
				cmd_vel_pub_bebop.publish(cmd_vel_bebop);			//Load the order to elevate
				correct_alt.publish(output_alt_pid);				//Publish the order
				ros::spinOnce();						//Refresh the topics
				ros::Duration(0.016).sleep();					//Little delay to accomplish the order
			}
	
			//HOVER
			printf("\n======== H O V E R ==========\n");
			cmd_vel_bebop.linear.x = 0;						//Puts the drone in HOVER setting everything to 0.
			cmd_vel_bebop.linear.y = 0;
			cmd_vel_bebop.linear.z = 0;

			cmd_vel_bebop.angular.x = 0;	
			cmd_vel_bebop.angular.y = 0;	
			cmd_vel_bebop.angular.z = 0;

			cmd_vel_pub_bebop.publish(cmd_vel_bebop);				//Load the order to hover
			ros::spinOnce();							//Refresh the topics
        		ros::spinOnce();
			ros::Duration(5).sleep();						//Stays on hover 5 seconds
			ros::spinOnce();

			printf("\n======== R O T A T I N G [0º] ==========\n");	

			cmd_vel_bebop.linear.x = 0;
			cmd_vel_bebop.linear.y = 0;
			cmd_vel_bebop.linear.z = 0;

			cmd_vel_bebop.angular.x = 0;	
			cmd_vel_bebop.angular.y = 0;	
			cmd_vel_bebop.angular.z = 0;

			_heading_t = constrainAngle(_heading);					//Reads the current heading
			_arc = calcArc(0,_heading_t);						//Calculates the arc to a fixed orientation (0º)
			//_fixed_heading = _heading;
	
			for(int i=0;i<800;i++)//60
			{
				_heading_t = constrainAngle(_heading);				//Reads the current heading
	        		_arc2 = calcArc(0,_heading_t);					//Calculates the arc to a fixed orientation (0º)
				_arc3 = _arc - _arc2;						//Calculates the difference between the target arc and the current one.
				cmd_vel_bebop.angular.z = pid_hdg.mis(_arc3,_arc);		//Data is sent to the PID control
				output_hdg_pid.data = cmd_vel_bebop.angular.z;			//Stream the data to the control heading topic
				/*printf(" <<%f>>\n",cmd_vel_bebop.angular.z);
				printf(" $$%f$$\n",_arc);
				printf(" !!%f!!\n",_arc2);*/
				cmd_vel_pub_bebop.publish(cmd_vel_bebop);			//Publish the order
				correct_hdg.publish(output_hdg_pid);				//Stream the datalogger data
				ros::spinOnce();						//Refresh the topics
				ros::Duration(0.016).sleep();					//Little delay to accomplish the order
			}
	
			//HOVER
			printf("\n======== H O V E R ==========\n");
			cmd_vel_bebop.linear.x = 0;						//Puts the drone in HOVER setting everything to 0.
			cmd_vel_bebop.linear.y = 0;
			cmd_vel_bebop.linear.z = 0;

			cmd_vel_bebop.angular.x = 0;	
			cmd_vel_bebop.angular.y = 0;	
			cmd_vel_bebop.angular.z = 0;

			cmd_vel_pub_bebop.publish(cmd_vel_bebop);				//Load the order to hover
			ros::spinOnce();							//Refresh the topics
        		ros::spinOnce();
			ros::Duration(5).sleep();						//Little delay to accomplish the order
			ros::spinOnce();	

			active = false;

		}
		if(test == 2)
		{
			//COMPLEX ROUTINE

			//HOVER
			printf("\n======== H O V E R ==========\n");
			cmd_vel_bebop.linear.x = 0;						//Puts the drone in HOVER setting everything to 0.
			cmd_vel_bebop.linear.y = 0;
			cmd_vel_bebop.linear.z = 0;

			cmd_vel_bebop.angular.x = 0;	
			cmd_vel_bebop.angular.y = 0;	
			cmd_vel_bebop.angular.z = 0;

			cmd_vel_pub_bebop.publish(cmd_vel_bebop);				//Load the order to hover
			ros::spinOnce();							//Refresh the topics
        		ros::spinOnce();
			ros::Duration(5).sleep();						//Little delay to accomplish the order
			ros::spinOnce();

			printf("\n========== D E S C E N D I N G  &  90 º ==========\n");	
			cmd_vel_bebop.linear.x = 0;
			cmd_vel_bebop.linear.y = 0;
			cmd_vel_bebop.linear.z = 0;

			cmd_vel_bebop.angular.x = 0;	
			cmd_vel_bebop.angular.y = 0;	
			cmd_vel_bebop.angular.z = 0;

			_heading_t = constrainAngle(_heading);					//Reads the current heading
			_arc = calcArc(90,_heading_t);						//Calculates the arc to a fixed orientation (90º)

			for(int i=0;i<720;i++)//60
			{
				_heading_t = constrainAngle(_heading);				//Reads the current heading
	        		_arc2 = calcArc(90,_heading_t);					//Calculates the arc to a fixed orientation (90º)
				_arc3 = _arc - _arc2;						//Calculates the difference between the target arc and the current one.
				cmd_vel_bebop.angular.z = pid_hdg.mis(_arc3,_arc);		//Data is sent to the PID heading control 
				output_hdg_pid.data = cmd_vel_bebop.angular.z;			//Stream the data to the control heading topic
				cmd_vel_bebop.linear.z = pid_alt.mis(alt.altitude,0.60);	//Data is sent to the PID altitude control (0.60 mts)
				output_alt_pid.data = cmd_vel_bebop.linear.z;			//Stream the data to the control altitude topic
				//printf("<<%f>>\n",cmd_vel_bebop.linear.z);
				cmd_vel_pub_bebop.publish(cmd_vel_bebop);			//Publish the order			
				correct_alt.publish(output_alt_pid);				//Stream the datalogger altitude data
				correct_hdg.publish(output_hdg_pid);				//Stream the datalogger heading data
				ros::spinOnce();						//Refresh the topics
				ros::Duration(0.016).sleep();					//Little delay to accomplish the order
			}
	
	
			printf("\n========== E L E V A T I N G  &  0 º ==========\n");	
			cmd_vel_bebop.linear.x = 0;
			cmd_vel_bebop.linear.y = 0;
			cmd_vel_bebop.linear.z = 0;

			cmd_vel_bebop.angular.x = 0;	
			cmd_vel_bebop.angular.y = 0;	
			cmd_vel_bebop.angular.z = 0;
	
			_heading_t = constrainAngle(_heading);					//Reads the current heading
			_arc = calcArc(0,_heading_t);						//Calculates the arc to a fixed orientation (0º)

			for(int i=0;i<720;i++)//60
			{
				_heading_t = constrainAngle(_heading);				//Reads the current heading
	        		_arc2 = calcArc(0,_heading_t);					//Calculates the arc to a fixed orientation (0º)
				_arc3 = _arc - _arc2;						//Calculates the difference between the target arc and the current one.
				cmd_vel_bebop.angular.z = pid_hdg.mis(_arc3,_arc);		//Data is sent to the PID heading control
				output_hdg_pid.data = cmd_vel_bebop.angular.z;			//Stream the data to the control heading topic
				cmd_vel_bebop.linear.z = pid_alt.mis(alt.altitude,1.10);	//Data is sent to the PID altitude control (1.10 mts)
				output_alt_pid.data = cmd_vel_bebop.linear.z;			//Stream the data to the control altitude topic
				//printf("<<%f>>\n",cmd_vel_bebop.linear.z);
				cmd_vel_pub_bebop.publish(cmd_vel_bebop);			//Publish the order
				correct_alt.publish(output_alt_pid);				//Stream the datalogger altitude data
				correct_hdg.publish(output_hdg_pid);				//Stream the datalogger heading data
				ros::spinOnce();						//Refresh the topics
				ros::Duration(0.016).sleep();					//Little delay to accomplish the order
			}
		}
		if(test == 3)
		{
			//CONTINUOUS ROTATING 	
			
			//HOVER
			printf("\n======== H O V E R ==========\n");
			cmd_vel_bebop.linear.x = 0;						//Puts the drone in HOVER setting everything to 0.
			cmd_vel_bebop.linear.y = 0;
			cmd_vel_bebop.linear.z = 0;

			cmd_vel_bebop.angular.x = 0;	
			cmd_vel_bebop.angular.y = 0;	
			cmd_vel_bebop.angular.z = 0;

			cmd_vel_pub_bebop.publish(cmd_vel_bebop);				//Load the order to hover
			ros::spinOnce();							//Refresh the topics
        		ros::spinOnce();
			ros::Duration(5).sleep();						//Little delay to accomplish the order
			ros::spinOnce();

			printf("\n========== R O T A T I N G ==========\n");	
			cmd_vel_bebop.linear.x = 0;
			cmd_vel_bebop.linear.y = 0;
			cmd_vel_bebop.linear.z = 0;

			cmd_vel_bebop.angular.x = 0;	
			cmd_vel_bebop.angular.y = 0;	
			cmd_vel_bebop.angular.z = 0;

			_heading_t = constrainAngle(_heading);					//Reads the current heading
			_arc = calcArc(_heading_t + 10,_heading_t);				//Calculates the arc to a fixed orientation (theta + 10º)

			while(n.ok())
			{
				_heading_t = constrainAngle(_heading);				//Reads the current heading
	        		_arc2 = calcArc(_heading_t + 30,_heading_t);			//Calculates the arc to a fixed orientation (theta + 30º)
				_arc3 = _arc - _arc2;						//Calculates the difference between the target arc and the current one.
				cmd_vel_bebop.angular.z = pid_hdg.mis(_arc3,_arc);		//Data is sent to the PID heading control
				output_hdg_pid.data = cmd_vel_bebop.angular.z;			//Stream the data to the control heading topic
				cmd_vel_bebop.linear.z = pid_alt.mis(alt.altitude,1.20);	//Data is sent to the PID altitude control (1.20 mts)
				output_alt_pid.data = cmd_vel_bebop.linear.z;			//Stream the data to the control altitude topic
				//printf("<<%f>>\n",cmd_vel_bebop.linear.z);
				cmd_vel_pub_bebop.publish(cmd_vel_bebop);			//Publish the order
				correct_alt.publish(output_alt_pid);				//Stream the datalogger altitude data
				correct_hdg.publish(output_hdg_pid);				//Stream the datalogger heading data
				ros::spinOnce();						//Refresh the topics
				ros::Duration(1).sleep();					//Little delay to accomplish the order
			}
		}
	}	


	printf("\n========== L A N D ==========\n");
	land_bebop.publish(msg_land);								//Publish the order to land the drone
	ros::spinOnce();									//Refresh the topics
	//r.sleep();
}


