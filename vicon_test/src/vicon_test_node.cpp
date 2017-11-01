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
	
double heading;								//Drone's heading
int btn_emergency;							//Emergency's Button to stop routine
geometry_msgs::Twist cmd_vel_bebop;					//Command Message 
ros::WallTime _time;							//Time of the system
ros::WallTime _actual_time;						//Actual time 
ros::WallTime _last_time;						//Last time captured
double _time_data;							//Raw time data 
double _time_secs;							//Time in seconds
double _time_wait;							//Time to wait in a waypoint


///////////////// V I C O N - B E B O P \\\\\\\\\\\\\\\\\\\\\\\\\\\\

struct drone								//Structure that describes the properties of the drone
{
	double _posX,_posY,_posZ;					//Position of the drone
	double _errorX,_errorY,_errorZ;					//Error of the position of the drone
	double _orX,_orY,_orZ,_orW;					//Orientation of the drone
	double _roll,_pitch,_yaw,_yawRAD;				//Roll,Pitch,Yaw (degrees), Yaw (radians)
	double _cmdX,_cmdY,_cmdZ,_cmdYAW;				//Command values 
	double rot_cmd_x,rot_cmd_y;					//Position in the rotated matrix
	double _velX,_velY,_velZ,_velYAW;				//Velocities
	double abs_x,abs_y;						//Absolute position in X and Y
	double angle_res,angle_arc;					//Angle resultant, angle of the arc
}bebop;

struct Point								//Structure to describe the Waypoints
{
	double x;
	double y;
};

void getJoyState(const sensor_msgs::Joy::ConstPtr& js)			//Function to obtain the data from the emergency button of the joystick
{
	btn_emergency = js->buttons[0];
}
void getBebopPos(const geometry_msgs::TransformStamped::ConstPtr& pos)	//Function to obtain the position from the vicon system
{
	bebop._posX = pos->transform.translation.x;			//Position in X
	bebop._posY = pos->transform.translation.y;			//Position in Y
	bebop._posZ = pos->transform.translation.z;			//Position in Z
	bebop._orX = pos->transform.rotation.x;				//Rotation in X
	bebop._orY = pos->transform.rotation.y;				//Rotation in Y
	bebop._orZ = pos->transform.rotation.z;				//Rotation in Z
	bebop._orW = pos->transform.rotation.w;				//Rotation in W

	tf::Quaternion q(pos->transform.rotation.x,pos->transform.rotation.y,pos->transform.rotation.z,pos->transform.rotation.w);
	tf::Matrix3x3 m(q);
	m.getRPY(bebop._roll,bebop._pitch,bebop._yawRAD);		//Get the Roll, Pitch, Yaw (Radians)
	bebop._yaw = bebop._yawRAD*(180/PI);				//Convert the Yaw (Radians) into Yaw (Degrees)
	heading = bebop._yaw;						//Set the heading of the drone
		
	bebop.abs_x = bebop._posX;					//Set the absolute position of the drone in X
	bebop.abs_y = bebop._posY;					//Set the absolute position of the drone in Y
}
void getAngtoCenter(void)						//Function to order the drone to face the center of the world
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
bool insideCircle()								//Function to avoid undesirable behavior of the drone when is near to the center
{	
	bool inside;
	double radio = sqrt(pow(bebop._posX, 2.0) + pow(bebop._posY, 2.0));	//Defining Deadzone
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
	ros::init(argc,argv,"bebop_vicon");					//Initiates the node
	ros::NodeHandle n;							//Creates the node handler
	ros::Subscriber joy_sub,vicon_sub;					//Creates the subscribers of the joystick and the vicon system
	ros::Publisher takeoff_bebop;						//Creates the publisher of the take-off command 
	ros::Publisher land_bebop;						//Creates the publisher of the land command
	ros::Publisher cmd_vel_pub_bebop;					//Creates the publisher of the movement command
	joy_sub = n.subscribe("/joy",1000,getJoyState);				//Initiates the Joystick Subscriber
	vicon_sub = n.subscribe("/vicon/Drone_LS/Drone_LS",1000,getBebopPos);	//Initiates the Vicon Subscriber to detect the Drone_LS object 
	cmd_vel_pub_bebop = n.advertise<geometry_msgs::Twist>("/bebop/cmd_vel",1000);	//Initiates the Command publisher into  the topics
	takeoff_bebop = n.advertise<std_msgs::Empty>("/bebop/takeoff",1000);	//Initiates the Take-Off publisher into the topics 
	land_bebop = n.advertise<std_msgs::Empty>("/bebop/land",1000);		//Initiates the Land publisher into the topics

	PID_h pid_hdg,pid_pos,pid_alt;						//Create a pid object for heading, position and altitude
	pid_hdg.gpro = 0.10;							//Setting gains
	pid_hdg.ginteg = 0.0;
	pid_hdg.gder = 0.00002;
	
	pid_pos.gpro = 1.353; // 1.3525
	pid_pos.ginteg = 0.0;
	pid_pos.gder = 0.0035;
	
	pid_alt.gpro = 3.5;
	pid_alt.ginteg = 0.0;
        pid_alt.gder = 0.0001;

	std_msgs::Empty msg_takeoff, msg_land;					//Message to command the drone to take-off and land 

	//Waypoints
	
	Point p[5];								//Setting the Waypoints
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


	double _arc,_arc2,_arc3;						//Arcs for the heading 
	
	//Rotation Control
	bool rot_control = true;						//Rotation Control to use a rotated matrix

	cmd_vel_bebop.linear.x = 0;						//Set every value to 0
	cmd_vel_bebop.linear.y = 0;
	cmd_vel_bebop.linear.z = 0;

	cmd_vel_bebop.angular.x = 0;	
	cmd_vel_bebop.angular.y = 0;	
	cmd_vel_bebop.angular.z = 0;

	ros::spinOnce();							//Refresh the topics	
		
	double hdg_target = 90;							//Initial target of the heading 
	bool correct_hdg = false;						//Indicates the state of the heading 
	bool correcting_hdg = false;
	bool inside_circle;
	
	
	heading = constrainAngle(heading);					//Get the current heading
	_arc = calcArc(hdg_target,heading); 					//Calculate the arc using as reference the initial target
		
	ros::Duration(2).sleep();						//Time necesary to setup the publishers	

	ros::Rate r(100);							//Setting the program to 100 Hz

	printf("\n========== T A K E O F F ==========\n");
	takeoff_bebop.publish(msg_takeoff);					//Take-off
	ros::Duration(4.5).sleep();						//Wait until the drone is flying
	
	/*printf("\n========== L A N D ==========\n");
	land_bebop.publish(msg_land);*/
	

	cmd_vel_pub_bebop.publish(cmd_vel_bebop);				//Set the drone to hover
	ros::Duration(0.5).sleep();						//Wait until the order is accomplished
	
	int waypoint = 0;							//Set the initial waypoint

	while(n.ok())								//Execute the program until <Ctrl + C> is pressed
	{
		if(btn_emergency)						//If the joystick's button is pressed land the drone and finish the program
		{
			ros::Duration(0.525).sleep();
			printf("\n========== L A N D [ J S ]==========\n");
			land_bebop.publish(msg_land);				//Land the drone
			break;
		}

		inside_circle = insideCircle();					//Check if the drone is inside of the deadzone 
		getAngtoCenter();						//Figure it out how to face to the center
		heading = constrainAngle(heading);				//Get the current heading
		_arc = calcArc(bebop.angle_arc,heading);			//Calculate the heading to face the center 
		ros::Duration(0.0001).sleep();					//A little delay to accomplish the order
		_arc2 = calcArc(bebop.angle_arc,heading);			//Calculate the heading to face the center  
		_arc3 = _arc - _arc2;						//Calculate the rest of the arc 

		if(!inside_circle)						//If the drone is outside the deadzone turn freely, otherwise lock
		{
			cmd_vel_bebop.angular.z = pid_hdg.mis(_arc3,_arc);
		}

		if(bebop._posX < (p[waypoint].x + 0.15)  && bebop._posX > (p[waypoint].x - 0.15) && bebop._posY < (p[waypoint].y + 0.15) && bebop._posY > (p[waypoint].y - 0.15) && _arc3 < (_arc + 10) && _arc3 > (_arc - 10 ))	//Reach every waypoints' zones
		{
			_time = ros::WallTime::now();					//Get the current time
			_actual_time = _time;						//Save the current time
			_time_secs = _actual_time.toSec() - _last_time.toSec();		//Get the time of the order in seconds
			_time_data += _time_secs;					//Sum the seconds to count every second inside the zone
			if(_time_data > 2)						//If the drone stays more than 2 seconds in the zone
			{
				/*cmd_vel_bebop.linear.x = 0;
				cmd_vel_bebop.linear.y = 0;
				cmd_vel_bebop.linear.z = 0;

				cmd_vel_bebop.angular.x = 0;	
				cmd_vel_bebop.angular.y = 0;	
				cmd_vel_bebop.angular.z = 0;*/
	
				cmd_vel_pub_bebop.publish(cmd_vel_bebop);		//Publish the order
		
				printf("\n ZONE [%d] REACHED \n",waypoint);
				ros::Duration(5).sleep();				//Stay 5 seconds in the zone
				_time_data = 0;
				waypoint = waypoint++;					//Change to the next waypoint 
				if (waypoint > 4)					//Once every waypoint is reached, start over 
				{
					waypoint = 0;
				}	
			}		
		}
		else
		{
			if (rot_control)						//Gives the option to use a rotated matrix
			{
				bebop._cmdX = pid_pos.mis(bebop._posX,p[waypoint].x);	//Data is sent to the position PID control in X
				bebop._cmdY = pid_pos.mis(bebop._posY,p[waypoint].y);	//Data is sent to the position PID control in Y
				//Adapting to the rotated matrix in X
				bebop.rot_cmd_x = bebop._cmdX*cos(bebop._yawRAD) + bebop._cmdY*sin(bebop._yawRAD); 
				//Adapting to the rotated matrix in Y
				bebop.rot_cmd_y = bebop._cmdX*sin(bebop._yawRAD) - bebop._cmdY*cos(bebop._yawRAD); 
					
					
				cmd_vel_bebop.linear.x = bebop.rot_cmd_x;		//Set the command data in X
				cmd_vel_bebop.linear.y = -bebop.rot_cmd_y;		//Set the command data in Y

			}
			else								//Does not uses a rotated matrix
			{
				cmd_vel_bebop.linear.x = pid_pos.mis(bebop._posX,p[waypoint].x);	//Set the command data in X
				cmd_vel_bebop.linear.y = pid_pos.mis(bebop._posY,p[waypoint].y);	//Set the command data in Y
			}
				_time_data = 0;						//Reset all time values
				_time_wait = 0;
		}	
			correct_hdg = false;						//Reset correct_hdg value
		
		cmd_vel_bebop.linear.z = pid_alt.mis(bebop._posZ,1.10);			//Set the Altitude PID to 1.10 mts
		cmd_vel_pub_bebop.publish(cmd_vel_bebop);				//Set the command to move the drone
		//ROS_INFO("Altura: %lf  Heading: %lf",bebop._posZ,bebop._yaw);
		ROS_INFO("X: %lf  Y: %lf CMDX: %lf  CMDY: %lf",bebop._posX,bebop._posY, cmd_vel_bebop.linear.x,cmd_vel_bebop.linear.y);
		ros::spinOnce();							//Refresh the topics
		r.sleep();								//Set the rate of the program
	}
	
	ros::Duration(2).sleep();							//Wait 2 seconds to let the program finish some process
	ros::spinOnce();								//Refresh the topics
	if(bebop._posZ > 0.60)								//If the drone still in the air, land it
	{
		ros::Duration(0.525).sleep();
		printf("\n========== L A N D [ A L T ]==========\n");	
		land_bebop.publish(msg_land);						//Land
	}
	return 0;
}



