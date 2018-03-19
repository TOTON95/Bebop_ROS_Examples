//Coded by Alexis Guijarro

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <iostream>
#include <string>

static const std::string OPENCV_ORIGINAL = "Bebop camera window";		//Original image 
static const std::string OPENCV_BINARY = "Binary window";			//Binary image

const double max_obj_area = 100000; 						//Maximum area reference of the object
const double min_obj_area = 690;						//Minimum area reference of the object
const double bebop_velocity = 0.07;						//Bebop general velocity
const double bebop_turn_velocity = 0.42;					//Bebop general turn velocity
const double area_distance = 3000000;						//The area determines the proximity of the camera to the object/noise
double dArea;									//Holds the general area of the objects at the imageconst

cv::Point2f drone_center;							//Represents the center of the image, also the Bebop's nose

cv_bridge::CvImagePtr cv_original;						//Original image container 

int iLowH = 150;								//Default HSV range values
int iHighH = 183;

int iLowS = 111;
int iHighS = 228;

int iLowV = 96;
int iHighV = 216;

int posX,posY = 0;								//Target Position
bool no_object = true;								//No Tracked object present

bool exit_from_cv = false;							//Variable to indicate to exit from OpenCV to ROS
int tracking_system_armed = 0;							//0 - Drone hovers, 1 - Drone move to the target

//AutoPicker HSV
bool auto_detect_hsv = false;
int pX,pY = 0;

std_msgs::Empty take_off,land;							//Variable to take_off and land
geometry_msgs::Twist cmd_vel_bebop;						//Variable that stores the linear velocity commands of Bebop

ros::Publisher takeoff_bebop;							//Sends the message to make the drone to take off 
ros::Publisher land_bebop;							//Sends the message to make the drone to land
ros::Publisher cmd_vel_pub_bebop;						//Sends the message to move the drone

void turn()
{
	cmd_vel_bebop.linear.x = 0;						//Turns the drone without translation
	cmd_vel_bebop.linear.y = 0;
	cmd_vel_bebop.linear.z = 0;

	cmd_vel_pub_bebop.publish(cmd_vel_bebop);				//Load the order to hover
}
void hover()
{
	cmd_vel_bebop.linear.x = 0;						//Puts the drone in HOVER
	cmd_vel_bebop.linear.y = 0; 
	cmd_vel_bebop.linear.z = 0;
	cmd_vel_bebop.angular.z = 0;

	cmd_vel_pub_bebop.publish(cmd_vel_bebop);				//Load the order to hover
}
void auto_pick_HSV(cv::Mat& image)
{
	cv::Vec3b temp= image.at<cv::Vec3b>(pY,pX);
	int blue = temp.val[0];
	int green = temp.val[1];
	int red = temp.val[2];
	
	cv::Mat tHSV;
	cv::cvtColor(image,tHSV,cv::COLOR_BGR2HSV);
	cv::Vec3b hsv = tHSV.at<cv::Vec3b>(pY,pX);
	int h = hsv.val[0];
	int s = hsv.val[1];
	int v = hsv.val[2];

	int range = 30;

	iLowH = h - range;
	iHighH = h + range;
	iLowS = s - range*2.5;
	iHighS = s + range*2.5;
	iLowV = v - range*3;
	iHighV = v + range*3;


	cv::setTrackbarPos("LowH", OPENCV_BINARY, iLowH);
	cv::setTrackbarPos("HighH",OPENCV_BINARY, iHighH);

	cv::setTrackbarPos("LowS", OPENCV_BINARY, iLowS);
	cv::setTrackbarPos("HighS", OPENCV_BINARY, iHighS);

	cv::setTrackbarPos("LowV", OPENCV_BINARY, iLowV);
	cv::setTrackbarPos("HighV", OPENCV_BINARY, iHighV);

	auto_detect_hsv = false;
}

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
     if  ( event == cv::EVENT_LBUTTONDOWN )
     {
	auto_detect_hsv = true;
	pX = x;
	pY = y;	
     }
}

void sort_blob_by_size(std::vector<cv::KeyPoint> k)
{
	cv::KeyPoint big_keypoint_by_size;
	if(k.size() > 0)
	{
		for(int i = 0; i<k.size();i++)
		{
			if(k[i].size > big_keypoint_by_size.size)
			{
				big_keypoint_by_size = k[i];
			}
		}
		posX = big_keypoint_by_size.pt.x;
		posY = big_keypoint_by_size.pt.y;
		no_object = false;
	}
	else
	{
		posX = drone_center.x;
		posY = drone_center.y;
		no_object = true;
	}
}

void c_drone(cv::Mat& image)
{
	cv::Point2f center;
	center.x = image.cols / 2;
	center.y = image.rows / 2;
	cv::circle(image, center, 5, cv::Scalar(0, 0, 255), -1);
	drone_center = center;
}

cv::Mat detect_blobs(cv::Mat& image)
{
	cv::SimpleBlobDetector::Params params;
	params.minThreshold = 10;
	params.maxThreshold = 200;
 
	params.filterByArea = true;
	params.minArea = min_obj_area;
	params.maxArea = max_obj_area;

	params.filterByColor = true;
	params.blobColor = 255;
 
	params.filterByCircularity = false;
 
	params.filterByConvexity = false;
 
	params.filterByInertia = false;
	cv::Ptr<cv::SimpleBlobDetector> d = cv::SimpleBlobDetector::create(params);
	std::vector<cv::KeyPoint> keypoints;
	cv::Mat cloned_image = image.clone();
	d->detect(cloned_image,keypoints);
	sort_blob_by_size(keypoints);
	cv::Mat result;
	cv::drawKeypoints( cloned_image, keypoints, result, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
	return result;
}

void discriminate_color(cv::Mat& image)
{
	cv::namedWindow(OPENCV_BINARY);						//Creating Control Interface
	cv::createTrackbar("LowH", OPENCV_BINARY, &iLowH, 180);
	cv::createTrackbar("HighH",OPENCV_BINARY, &iHighH, 180);

	cv::createTrackbar("LowS", OPENCV_BINARY, &iLowS, 255);
	cv::createTrackbar("HighS", OPENCV_BINARY, &iHighS, 255);

	cv::createTrackbar("LowV", OPENCV_BINARY, &iLowV, 255);
	cv::createTrackbar("HighV", OPENCV_BINARY, &iHighV, 255);
	
	if(auto_detect_hsv){auto_pick_HSV(image);}				//AutoPicker HSV 

	cv::Mat imgHSV;								//HSV Image Container
	cv::cvtColor(cv_original->image, imgHSV, cv::COLOR_BGR2HSV);		//Convert BGR to HSV Color Space

	cv::Mat imgThresholded;
	cv::inRange(imgHSV, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), imgThresholded);
	//ROS_INFO("LowH: %d LowS: %d LowV: %d HighH: %d HighS: %d HighV: %d",iLowH, iLowS, iLowV,iHighH, iHighS, iHighV);

	cv::erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));	//Morphology Operations
	cv::dilate(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
	cv::erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
	cv::dilate(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

	cv::blur(imgThresholded,imgThresholded,cv::Size(5,5));			//Basic Filtering
	
	cv::Mat cntr = imgThresholded.clone();					//Contours container
	std::vector<std::vector<cv::Point> > contours;				//Contours vector
	cv::findContours(cntr, contours, CV_RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);	//Find contours
	cv::drawContours(cntr, contours, -1, cv::Scalar(255, 255, 255), -1);		//Draw contours
	std::vector<std::vector<cv::Point> > convexHulls(contours.size());
	for (unsigned int i = 0; i < contours.size(); i++) 			//Find Convex Hulls
	{
		cv::convexHull(contours[i], convexHulls[i]);
	}
	cv::Mat conv(cntr.size(), CV_8UC3, cv::Scalar(0, 0, 0));		//Convex Hulls container
	cv::drawContours(conv, convexHulls, -1, cv::Scalar(255, 255, 255), -1); //Draw Convex Hulls
	cv::dilate(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(10, 10)));	//Closing gaps
	cv::erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(10, 10)));
	cv:cvtColor(conv,conv,cv::COLOR_BGR2GRAY);				//Convert to 1-channel image
	cv::Mat blobs = detect_blobs(conv);					//Detect the blobs in the image, returns the image with detected blobs
	cv::imshow(OPENCV_BINARY,blobs);					//Show binary image with blobs
	cv::Moments mu = moments(imgThresholded);						//Calculate moments
	dArea=mu.m00;								//Area that determines if the drone is near the target and stop it
	contours.clear();
	convexHulls.clear();
}

void move_drone(cv::Mat& image)
{
	

	if (posX >= 0 && posY >= 0)
	{
		if (posX < drone_center.x)
		{
			cmd_vel_bebop.angular.z = bebop_turn_velocity;						
			std::cout << "Move Left" << std::endl;
		}
		if (posX > drone_center.x)
		{
			cmd_vel_bebop.angular.z = -bebop_turn_velocity;
			std::cout << "Move Right" << std::endl;
		}
		if (posY > drone_center.y)
		{
			cmd_vel_bebop.linear.z = -bebop_velocity;
			std::cout << "Move Downwards" << std::endl;
		}
		if (posY < drone_center.y)
		{
			cmd_vel_bebop.linear.z = bebop_velocity;
			std::cout << "Move Upwards" << std::endl;
		}
		if(tracking_system_armed)
		{
			cv::circle(image, cv::Point(posX, posY), 8, cv::Scalar(0, 255, 0), -1);
		}
		if(!tracking_system_armed)
		{
			cv::circle(image, cv::Point(posX, posY), 8, cv::Scalar(0, 255, 255), -1);
		}
		
	}
	if(tracking_system_armed)
	{
		int intFontFace = CV_FONT_HERSHEY_SIMPLEX;
		//double dblFontScale = 0.75;
		double dblFontScale = 1;
		int intFontThickness = 2;
		//cv::putText(image, "Tracking System ARMED", cv::Point(0,image.cols-175), intFontFace, dblFontScale, cv::Scalar(0,255,0), intFontThickness);
		cv::putText(image, "Tracking System ARMED", cv::Point(0,image.cols/2), intFontFace, dblFontScale, cv::Scalar(0,255,0), intFontThickness);
		if(dArea < area_distance && !no_object)
		{
			cmd_vel_bebop.linear.x = bebop_velocity;
			cmd_vel_pub_bebop.publish(cmd_vel_bebop);
			//cv::putText(image, "<Following>", cv::Point(image.rows-20,image.cols-175), intFontFace, dblFontScale, cv::Scalar(0,255,255), intFontThickness);
		cv::putText(image, "<Following>", cv::Point(image.rows-20,image.cols/2), intFontFace, dblFontScale, cv::Scalar(0,255,255), intFontThickness);
		}
		else if(!(dArea < area_distance) && !no_object){turn();}
		else if(!(dArea < area_distance) && no_object){hover();}
	}
	else
	{
		hover();	
	}
	
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	try		
	{
		cv_original = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);	//Copying the image and encoding it into BGR according to opencv default settings
		discriminate_color(cv_original->image);				//Discriminate colors
		cv::Mat final_img = cv_original->image.clone();			//Clone the original image
		c_drone(final_img);						//Figures out bebop's center and draws a circle
		move_drone(final_img);						//Moves Drone
		cv::imshow(OPENCV_ORIGINAL,final_img);				//Show the original image
		int key = cv::waitKey(30);					//Contains the key value 
		if(key == 27)							//Press ESC to exit
		{
			exit_from_cv = true; 
			land_bebop.publish(land);
		}				
		if(key == 'k')		
		{
			tracking_system_armed = 1 - tracking_system_armed;
	
			if(tracking_system_armed)
			{
				ROS_INFO("TRACKING SYSTEM ARMED!!!");
			}
			else
			{
				ROS_INFO("TRACKING SYSTEM DISARMED!!!");
			}
		}
		if(key == ' ')
		{
			ROS_INFO("TAKE-OFF!!!");
			takeoff_bebop.publish(take_off);
		}			
		if(key == 'b')
		{
			ROS_INFO("LAND!!!");
			land_bebop.publish(land);
		}
	
		
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());			//Handleling the Exception
		return;
	}			
}
int main(int argc, char** argv)
{
	ros::init(argc,argv,"bebop_color_follower");				//Initialize the ROS node
	ros::NodeHandle nh_;							//Create the Node Handler
	image_transport::ImageTransport it_(nh_);				//Special message to contain the image
	image_transport::Subscriber image_sub_;					//Special subscriber to obtain the image
	//image_sub_= it_.subscribe("/usb_cam/image_raw",1,imageCallback);	//Subscribe to the Bebop image topic
	image_sub_= it_.subscribe("/bebop/image_raw",1,imageCallback);
	takeoff_bebop = nh_.advertise<std_msgs::Empty>("/bebop/takeoff",1000);		//Publish data to the take-off topic
	land_bebop = nh_.advertise<std_msgs::Empty>("/bebop/land",1000);		//Publish data to the land topic
	cmd_vel_pub_bebop = nh_.advertise<geometry_msgs::Twist>("/bebop/cmd_vel",1000);	//Publish data to the movement topic
	cv::namedWindow(OPENCV_ORIGINAL);					//Create window to visualize the original image
	cv::setMouseCallback(OPENCV_ORIGINAL, CallBackFunc, NULL);		//Receive info from the mouse
	//TODO Resolve Segmentation Fault issue due to publishers 
	while(nh_.ok())								//Asks if the node still alive
	{	
		ros::spinOnce();						//Refresh ROS's topics once
		if(exit_from_cv){break;}					//Exit using while focus any opencv window (ESC key)	
	}
	ROS_INFO("EXITING...");
	cv::destroyWindow(OPENCV_ORIGINAL);					//Destroy Original Window
	cv::destroyWindow(OPENCV_BINARY);					//Destroy Binary Window
	return 0;
}
