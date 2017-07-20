#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>
#include <iostream>

static const std::string OPENCV_WINDOW = "Bebop camera window";

class ImageBebop
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;
public:
	ImageBebop()
	  : it_(nh_)
        {
	  // Subscrive to input video feed and publish output video feed
	  image_sub_ = it_.subscribe("/bebop/image_raw", 1,&ImageBebop::imageCb, this);
	  //image_pub_ = it_.advertise("/image_converter/output_video", 1);
	  cv::namedWindow(OPENCV_WINDOW);
	}
	
	~ImageBebop()
	{
	 cv::destroyWindow(OPENCV_WINDOW);
	}
	
	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{
		cv_bridge::CvImagePtr cv_ptr;
	        try
		{
		  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception& e)
		{
		  ROS_ERROR("cv_bridge exception: %s", e.what());
		  return;
		}
		

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

		// Update GUI Window
		cv::imshow(OPENCV_WINDOW, cv_ptr->image);
		cv::waitKey(3);
	
		

		// Output modified video stream
		//image_pub_.publish(cv_ptr->toImageMsg());
	}
};

int main(int argc, char** argv)
{
	ros::init(argc,argv,"bebop_cv_camera");
	ImageBebop ic;
	ros::spin();
	return 0;
}
