//Coded by Alexis Guijarro

#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_datatypes.h>
#include <Bebop_waypoints/ArcDrone.h>
#include <Bebop_waypoints/Waypoints.h>
#include <Bebop_waypoints/Waypoint.h>
#include <ros/ros.h>
#include <stdlib.h>
#include <time.h>
#include <iostream>
#include <vector>

#define PI 3.141592653589793238462

double heading;                                                         //Drone's heading
int btn_emergency;                                                      //Physical emergency's button
geometry_msgs::Twist cmd_vel_bebop;                                     //Contains the velocity information to be sent
bool on_live = false;							//The option to modify waypoints from topics
double ce_hdg, ce_pos_X, ce_pos_Y, ce_alt;                              //Control efforts delivered by the PD controller
std_msgs::Float64 st_pos_X,st_pos_Y,st_alt,st_hdg;                      //State variables
std_msgs::Float64 stp_pos_X,stp_pos_Y,stp_alt,stp_hdg;                  //Setpoint variables
Bebop_waypoints::Waypoints waypoints;	                                //Vector with waypoints
unsigned int c_wps = 0;							//Current waypoint index
double goal_bound = 0.05;						//Boundaries of the goal
Bebop_waypoints::Waypoint t_wps;					//Current target waypoint
double hdg_target; 	                                          	//Heading target
int wps;								//Number of waypoints
std::vector<double> x,y,z;						//Parameters vectors
std::vector<double> headings;						//Headings vectors
double time_goal;							//Time to stay at the waypoint
ros::WallTime _time;                                                    //Time of the system
ros::WallTime _actual_time;                                             //Actual time 
ros::WallTime _last_time;                                               //Last time captured
double _time_data;                                                      //Raw time data 
double _time_secs;                                                      //Time in seconds
double _time_wait;  

struct v_object                                                         //Structure that describes the properties of the object
{
        double _posX,_posY,_posZ;                                       //Position of the drone
        double _errorX,_errorY,_errorZ;                                 //Error of the position of the drone
        double _orX,_orY,_orZ,_orW;                                     //Orientation of the drone
        double _roll,_pitch,_yaw,_yawRAD;                               //Roll,Pitch,Yaw (degrees), Yaw (radians)
        double _cmdX,_cmdY,_cmdZ,_cmdYAW;                               //Command values
        double rot_cmd_x,rot_cmd_y;                                     //Position in the rotated matrix
        double _velX,_velY,_velZ,_velYAW;                               //Velocities
        double abs_x,abs_y;                                             //Absolute position in X and Y
        double angle_res,angle_arc;                                     //Angle resultant, angle of the arc
}bebop;

void getJoyState(const sensor_msgs::Joy::ConstPtr& js)                  //Function to obtain the data from the joystick
{
        btn_emergency = js->buttons[0];
}

void getBebopPos(const geometry_msgs::TransformStamped::ConstPtr& pos)  //Function to obtain the position from the vicon system
{
        bebop._posX = pos->transform.translation.x;                     //Position in X
        bebop._posY = pos->transform.translation.y;                     //Position in Y
        bebop._posZ = pos->transform.translation.z;                     //Position in Z
        bebop._orX = pos->transform.rotation.x;                         //Rotation in X
        bebop._orY = pos->transform.rotation.y;                         //Rotation in Y
        bebop._orZ = pos->transform.rotation.z;                         //Rotation in Z
        bebop._orW = pos->transform.rotation.w;                         //Rotation in W

        tf::Quaternion q(pos->transform.rotation.x,pos->transform.rotation.y,pos->transform.rotation.z,pos->transform.rotation.w);
        tf::Matrix3x3 m(q);
        m.getRPY(bebop._roll,bebop._pitch,bebop._yawRAD);               //Get the Roll, Pitch, Yaw (Radians)
        bebop._yaw = bebop._yawRAD*(180/PI);                            //Convert the Yaw (Radians) into Yaw (Degrees)
        heading = bebop._yaw;                                           //Set the heading of the drone


        bebop.abs_x = bebop._posX;                                      //Set the absolute position of the drone in X
        bebop.abs_y = bebop._posY;                                      //Set the absolute position of the drone in Y
}

//Avoids problems with the heading of the motion capture system

double GetAngleDifference(double from, double to)
{
        double difference = to - from;
        while (difference < -180) difference += 360;
        while (difference > 180) difference -= 360;
        return difference;
}

//Get Control Efforts

void getEffort_pos_X(const std_msgs::Float64::ConstPtr& msg)
{
        ce_pos_X = msg->data;
}
void getEffort_pos_Y(const std_msgs::Float64::ConstPtr& msg)
{
        ce_pos_Y = msg->data;
}
void getEffort_alt(const std_msgs::Float64::ConstPtr& msg)
{
        ce_alt = msg->data;
}
void getEffort_hdg(const std_msgs::Float64::ConstPtr& msg)
{
        ce_hdg = -msg->data;
}

//Set waypoints from parameters
bool setWayPoints()
{
	if(x.size() != wps || y.size() != wps || z.size() != wps || headings.size() != wps)
	{
		ROS_ERROR("\nNumber of waypoints does not match with number of positions!\n");
		return true;
	}
	
	

	for(int k=0;k<wps;k++)
	{
		Bebop_waypoints::Waypoint z0;
		z0.wp.x = x[k];
		z0.wp.y = y[k];
		z0.wp.z = z[k];
		z0.hdg = headings[k];

		waypoints.wps.push_back(z0);
	}

	return false;
}

//Get Waypoints from existent topic
void getWaypoints(const Bebop_waypoints::Waypoints::ConstPtr& msg)
{
	waypoints.wps = msg -> wps;
}

//Procedure to reach every waypoint
bool getWayPointInfo()
{	
	if(c_wps > (waypoints.wps.size()-1))
	{
		ROS_INFO("\n\n\nReached end of the routine\n\n\n");
		return false;
	}

	t_wps = waypoints.wps[c_wps];						//Assign stored waypoint to current waypoint

	bool x_no_top_reached = bebop._posX < (waypoints.wps[c_wps].wp.x + goal_bound);
	bool x_no_bottom_reached = bebop._posX > (waypoints.wps[c_wps].wp.x - goal_bound);
	bool y_no_top_reached = bebop._posY < (waypoints.wps[c_wps].wp.y + goal_bound);
        bool y_no_bottom_reached = bebop._posY > (waypoints.wps[c_wps].wp.y - goal_bound);
	bool z_no_top_reached = bebop._posZ < (waypoints.wps[c_wps].wp.z + goal_bound);
        bool z_no_bottom_reached = bebop._posZ > (waypoints.wps[c_wps].wp.z - goal_bound);

	//std::cout<<x_no_top_reached<<" "<<x_no_bottom_reached<<" "<<y_no_top_reached<<" "<<y_no_bottom_reached<<" "<<z_no_top_reached<<" "<<z_no_bottom_reached<<std::endl;

	if(x_no_top_reached && x_no_bottom_reached && y_no_top_reached && y_no_bottom_reached && z_no_top_reached && z_no_bottom_reached)
	{
		_time = ros::WallTime::now();                                   //Get the current time
                _actual_time = _time;                                           //Save the current time
                _time_secs = _actual_time.toSec() - _last_time.toSec();         //Get the time of the order in seconds
                _time_data += _time_secs;                                       //Sum the seconds to count every second inside the zone
                if(_time_data > time_goal)                                      //If the drone stays more than x seconds in the zone
                {
			_time_data = 0;
			ROS_INFO("\n<--Waypoint %d reached-->\n",c_wps);
			c_wps++;
		}
	}
	return true;
}

int main(int argc, char** argv)
{
        ros::init(argc,argv,"bebop_waypoints");			        //Initiates the ROS node
        ros::NodeHandle n;                                              //Creates the node handle
        ros::Subscriber joy_sub,bebop_sub;                              //Joystick and Vicon subs
        ros::Subscriber pid_pos_X,pid_pos_Y,pid_hdg,pid_alt;            //PID controllers subs
	ros::Subscriber wps_sub;					//Waypoints Sub
        ros::Publisher state_pos_X,state_pos_Y,state_hdg,state_alt;     //Current state of the drone
        ros::Publisher sp_pos_X,sp_pos_Y,sp_hdg,sp_alt;                 //Set the goal of the drone
        ros::Publisher tko,land;                                        //Take-off and landing publisher
        ros::Publisher cmd_vel;                                         //Velocity command of the drone

	//Getting parameters
	bool load_param = true;
	if(!n.getParam("waypoints",wps))
	{
		ROS_ERROR("\nFailed to load the waypoints parameters\n");
		load_param = false;
	}
	if(!n.getParam("X",x))
        {
                ROS_ERROR("\nFailed to load the X parameters\n");
		load_param = false;
        }
	if(!n.getParam("Y",y))
        {
                ROS_ERROR("\nFailed to load the Y parameters\n");
		load_param = false;
        }
	if(!n.getParam("Z",z))
        {
                ROS_ERROR("\nFailed to load the Z parameters\n");
                load_param = false;
        }
	if(!n.getParam("Hdg",headings))
        {
                ROS_ERROR("\nFailed to load the Headings parameters\n");
                load_param = false;
        }
	n.param("Time_goal",time_goal,0.05);

	//Load the parameters
	if(load_param)
	{
		if(setWayPoints())
			return -1;
	}
	else
	{
		ROS_ERROR("\nStarting at the center of the world, ready to receive waypoints!\n");
		t_wps.wp.x = 0;
		t_wps.wp.y = 0;
		t_wps.wp.z = 1.05;
		t_wps.hdg = 0;
		waypoints.wps.push_back(t_wps);
		on_live = true;
	}


        //Subscribers of the ROS node
	joy_sub = n.subscribe("/joy",1000,getJoyState);
        bebop_sub = n.subscribe("/vicon/BEBOP_1_11_2_18/BEBOP_1_11_2_18",1000,getBebopPos);
        pid_pos_X = n.subscribe("/bebop_wps/control_effort_pos_X",1000,getEffort_pos_X);
        pid_pos_Y = n.subscribe("/bebop_wps/control_effort_pos_Y",1000,getEffort_pos_Y);
        pid_alt = n.subscribe("/bebop_wps/control_effort_alt",1000,getEffort_alt);
        pid_hdg = n.subscribe("/bebop_wps/control_effort_hdg",1000,getEffort_hdg);
	wps_sub = n.subscribe("/bebop_wps/waypoints",100,getWaypoints);

        //Publishers of the ROS node
        tko = n.advertise<std_msgs::Empty>("/bebop/takeoff",1000);
        land = n.advertise<std_msgs::Empty>("/bebop/land",1000);
	cmd_vel = n.advertise<geometry_msgs::Twist>("/bebop/cmd_vel",1000);
        state_pos_X = n.advertise<std_msgs::Float64>("/bebop_wps/state_pos_X",1000);
        state_pos_Y = n.advertise<std_msgs::Float64>("/bebop_wps/state_pos_Y",1000);
        state_alt = n.advertise<std_msgs::Float64>("/bebop_wps/state_alt",1000);
        state_hdg = n.advertise<std_msgs::Float64>("/bebop_wps/state_hdg",1000);
        sp_pos_X = n.advertise<std_msgs::Float64>("/bebop_wps/setpoint_pos_X",1000);
        sp_pos_Y = n.advertise<std_msgs::Float64>("/bebop_wps/setpoint_pos_Y",1000);
        sp_alt = n.advertise<std_msgs::Float64>("/bebop_wps/setpoint_alt",1000);
        sp_hdg = n.advertise<std_msgs::Float64>("/bebop_wps/setpoint_hdg",1000);

	std_msgs::Empty msg_tko,msg_land;                                //Msgs to take-off and land

        //Making sure to set every velocity to 0
        cmd_vel_bebop.linear.x = 0;
        cmd_vel_bebop.linear.y = 0;
        cmd_vel_bebop.linear.z = 0;
        cmd_vel_bebop.angular.x = 0;
        cmd_vel_bebop.angular.y = 0;
        cmd_vel_bebop.angular.z = 0;

        ros::spinOnce();                                                 //Refresh the topics

        ros::Duration(2).sleep();                                        //Time necessary to setup
        ros::Rate r(100);                                                //Set node at 100Hz

        printf("\n========== T A K E O F F ==========\n");
        tko.publish(msg_tko);                                            //Take-off
        ros::Duration(4.5).sleep();                                      //Wait until the drone elevates

        //Stops the node once that <Ctrl + C > is pressed
        while(n.ok())
        {
                //Lands the drone if the joystick's button is pressed
                if(btn_emergency)
                {
                        ros::Duration(0.525).sleep();
                        printf("\n========== L A N D [ J S ]==========\n");
                        land.publish(msg_land);                           //Land the drone
                        break;
                }
		
		if(!getWayPointInfo() && !on_live)			  //Get Waypoint information
		{
			ros::Duration(0.525).sleep();
                        printf("\n========== L A N D [ E N D ]==========\n");
                        land.publish(msg_land);                           //Land the drone if end is reached
                        break;
		}

                //Wrapping the angle
                double diff = GetAngleDifference(heading,t_wps.hdg);

                //Updating Heading PID Controller
                st_hdg.data=diff;
                stp_hdg.data=0;
                ros::spinOnce();

		state_hdg.publish(st_hdg);
                sp_hdg.publish(stp_hdg);
                ros::Duration(0.0001).sleep();

                ros::spinOnce();

                //Set heading velocity
                cmd_vel_bebop.angular.z = ce_hdg;

                //Updating Position PID Controller
                ros::spinOnce();

                st_pos_X.data=bebop._posX;
                stp_pos_X.data=t_wps.wp.x;
                state_pos_X.publish(st_pos_X);
                sp_pos_X.publish(stp_pos_X);
                ros::spinOnce();
                bebop._cmdX = ce_pos_X;

                st_pos_Y.data=bebop._posY;
                stp_pos_Y.data=t_wps.wp.y;
                state_pos_Y.publish(st_pos_Y);
                sp_pos_Y.publish(stp_pos_Y);
                ros::spinOnce();
                bebop._cmdY = ce_pos_Y;

                //Calculating the rotation matrix of the drone
                bebop.rot_cmd_x = bebop._cmdX*cos(constrainAngle(heading) * 0.0174533) + bebop._cmdY*sin(constrainAngle(heading) * 0.0174533);
                bebop.rot_cmd_y = bebop._cmdX*sin(constrainAngle(heading) * 0.0174533) - bebop._cmdY*cos(constrainAngle(heading) * 0.0174533);

                //Set X and Y velocities
                cmd_vel_bebop.linear.x = bebop.rot_cmd_x;
                cmd_vel_bebop.linear.y = -bebop.rot_cmd_y;

                //Updating Altittude PID Controller
                st_alt.data = bebop._posZ;
                stp_alt.data = t_wps.wp.z;
                state_alt.publish(st_alt);
                sp_alt.publish(stp_alt);
                ros::spinOnce();

		//Set velocity in Z
                cmd_vel_bebop.linear.z = ce_alt;

                //Send the velocity command
                cmd_vel.publish(cmd_vel_bebop);

                ROS_INFO("X: %lf  Y: %lf HDG: %lf CMDX: %lf  CMDY: %lf CMDHDG: %lf WAYPOINT: %i",bebop._posX,bebop._posY, heading, cmd_vel_bebop.linear.x,cmd_vel_bebop.linear.y,cmd_vel_bebop.angular.z,c_wps);

                ros::spinOnce();

                r.sleep();

        }

        ros::Duration(2).sleep();                                       //Wait 2 Seconds to finish
        ros::spinOnce();

        //Land the drone if it still flying
        while(bebop._posZ > 0.60)
        {
		ros::spinOnce();
                ros::Duration(0.525).sleep();
                printf("\n========== L A N D [ A L T ]==========\n");
                land.publish(msg_land);
        }

        return 0;
}

