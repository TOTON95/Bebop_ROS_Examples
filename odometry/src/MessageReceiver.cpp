//Coded by Alexis Guijarro
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <iostream>


//Callback to obtain the odometry data
void messageCallback(const nav_msgs::Odometry::ConstPtr& msg)
{	
	double var1 = msg->pose.pose.position.x;						//Position X of the drone
	double var2 = msg->pose.pose.position.y;						//Position Y of the drone 
	double var3 = msg->pose.pose.position.z;						//Position Z of the drone
	double var4 = msg->pose.pose.orientation.x;						//Orientation X of the drone
	double var5 = msg->pose.pose.orientation.y;						//Orientation Y of the drone
	double var6 = msg->pose.pose.orientation.z;						//Orientation Z of the drone
	double var7 = msg->pose.pose.orientation.w;						//Orientation W of the drone
	double var8 = msg->twist.twist.linear.x;						//Linear velocity in X axis
	double var9 = msg->twist.twist.linear.y;						//Linear velocity in Y axis
	double var10 = msg->twist.twist.linear.z;						//Linear velocity in Z axis
	double var11 = msg->twist.twist.angular.x;						//Angular velocity in X axis
	double var12 = msg->twist.twist.angular.y;						//Angular velocity in Y axis
	double var13 = msg->twist.twist.angular.z;						//Angular velocity in Z axis
	//ROS_INFO("\n Position:	Orientation:	Linear:	        Angular:\n x: %f 	x: %f 	x:  %f    x:\n y: %f \n z: %f \n",var1,var4,var8,var11,var2,var3);
	/*std::cout<<"\n Position:	Orientation:	  Linear:     Angular:\n x: "<<var1<<" 	        x: "<<var4<<" 	  x:  "<<var8<<"       x: "<<var11<<"\n y: "<<var2<<" 	        y: "<<var5<<" 	  y:  "<<var9<<"       y: "<<var12<<"\n z: "<<var3<<" 	        z: "<<var6<<" 	  z:  "<<var10<<"      z: "<<var13<<"\n                w: "<<var7<<""<<std::endl;*/
	std::cout<<"========================================\n"<<std::endl;			//Print the data into the console	
	std::cout<<" Position:	Orientation:\n x: "<<var1<<" 		x: "<<var4<<std::endl;
	std::cout<<" y: "<<var2<<" 		y: "<<var5<<std::endl;
	std::cout<<" z: "<<var3<<" 		z: "<<var6<<std::endl;
	std::cout<<"    		w: "<<var7<<"\n"<<std::endl;
	std::cout<<" Linear:	Angular:\n x: "<<var8<<" 		x: "<<var11<<std::endl;
	std::cout<<" y: "<<var9<<" 		y: "<<var12<<std::endl;
	std::cout<<" z: "<<var10<<" 		z: "<<var13<<"\n"<<std::endl;
	std::cout<<"========================================\n"<<std::endl;
	std::cout.flush();									//Erase the data of the screen
}
	
//Main Program
int main(int argc, char **argv)
{
	
	ros::init(argc,argv,"MessageReceiver");							//Initiates the node
	ros::NodeHandle n;									//Create the node handle
	ros::Subscriber sub = n.subscribe("/bebop/odom", 1000,messageCallback);			//Subscribe to the Odometry topic
	ros::spin();										//Refresh the topics
	return 0;										//End the program
}
	
