#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>
#include <iostream>

static const std::string OPENCV_WINDOW = "Bebop camera window";					//Name of the Window

// Class to Obtain the image from Bebop's camera
class ImageBebop
{
	ros::NodeHandle nh_;									//Create the Node Handler
	image_transport::ImageTransport it_;							//Special message to contain the image
	image_transport::Subscriber image_sub_;							//Special subscriber to obtain the image
	image_transport::Publisher image_pub_;							//Special publisher to publish images
public:
	ImageBebop()										//Class Constructor
	  : it_(nh_)
        {
	  // Subscribe to input video feed and publish output video feed
	  image_sub_ = it_.subscribe("/bebop/image_raw", 1,&ImageBebop::imageCb, this);
	  //image_pub_ = it_.advertise("/image_converter/output_video", 1);
	  cv::namedWindow(OPENCV_WINDOW);
	}
	
	~ImageBebop()										//Class Destructor
	{
	 cv::destroyWindow(OPENCV_WINDOW);							//Destroys the window
	}
	
	void imageCb(const sensor_msgs::ImageConstPtr& msg)					//Function to obtain the image				
	{
		cv_bridge::CvImagePtr cv_ptr;							//Image container
	        try		
		{
		  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);	//Copying the image and encoding it into BGR according to opencv default settings
		}
		catch (cv_bridge::Exception& e)
		{
		  ROS_ERROR("cv_bridge exception: %s", e.what());				//Handleling the Exception
		  return;
		}
		

		//====================!!!  Currently Disabled !!! ==========================\\ 

		/*cv::VideoCapture cap;
		//double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH);
	        //double dHeigth = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
		cv::Size framesize(static_cast<int>(1080), static_cast<int>(720));
		cv::VideoWriter oVideoWriter("Prueba.avi", CV_FOURCC('P', 'I', 'M', '1'), 20, framesize, true);
		
		if (!oVideoWriter.isOpened())
		{
		
		}
		
		bool bSuccess = cap.read(cv_ptr->image);

		if (!bSuccess)
		{
			
		}*/

		
		// Draw an example circle on the video stream
		/*if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
		  cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));*/

		//====================!!!  Currently Disabled !!! ==========================\\ 

		// Update GUI Window
		cv::imshow(OPENCV_WINDOW, cv_ptr->image);
		cv::waitKey(3);									//Checks at least every 3 ms if a key is pressed
	
		

		//====================!!!  Currently Disabled !!! ==========================\\
		// Output modified video stream
		//image_pub_.publish(cv_ptr->toImageMsg());
		//====================!!!  Currently Disabled !!! ==========================\\
	
	}
};

//Main Program
int main(int argc, char** argv)
{
	ros::init(argc,argv,"bebop_cv_camera");							//Initiates the node
	ImageBebop ic;										//Starts the image streaming
	ros::spin();										//Refresh the topic
	return 0;										//End the program
}
