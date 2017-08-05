#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";


class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
//    image_sub_ = it_.subscribe("/pioneer3at_ros/camera_ros/image", 1,
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
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

    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
      
      //cv::Mat gray;
      //cv_bridge::CvImage  gray;
      //cv::cvtColor(cv_ptr->image, gray.image, CV_BGR2GRAY);
 
 //////////////////////////////////////////////////////// 
      cv_bridge::CvImage preview, nextview;
                        
      cv::cvtColor(cv_ptr->image, nextview.image, CV_BGR2GRAY);  
         
      //2color white/black
      //cv::threshold(gray.image, gray.image, 127, 255, cv::THRESH_BINARY); 
		
			if(&preview.image != 0){	
			
				std::vector<cv::KeyPoint>  keypoints;
  	   	std::vector<cv::Point2f>   prepoints;
  	    	
  	 	  cv::GoodFeaturesToTrackDetector detector(100, 0.05, 3);
  	 	  detector.detect(preview.image, keypoints);
  	  	  
  	 	  for(std::vector<cv::KeyPoint>::iterator itk = keypoints.begin(); itk != keypoints.end(); ++itk){
  	 	  	prepoints.push_back(itk->pt);
  	 	  }
  
  			std::vector<cv::Point2f>  newpoints;
				std::vector<unsigned char>  status;
				std::vector<float>   errors;
				
				cv::calcOpticalFlowPyrLK(preview.image, nextview.image, prepoints, newpoints, status, errors, cv::Size(21,21), 3, cvTermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.05), 0);
				}
  	    
			cv::cvtColor(nextview.image, preview.image, CV_BGR2GRAY);
//////////////////////////////////////////////////////			

    // Update GUI Window
    //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::imshow(OPENCV_WINDOW, nextview.image);
    cv::waitKey(3);

    // Output modified video stream
    //image_pub_.publish(cv_ptr->toImageMsg());
     
     //When you with publish color
     //image_pub_.publish(gray.toImageMsg());
     
     //When you with publish mono
     image_pub_.publish(cv_bridge::CvImage(std_msgs::Header(),"mono8",nextview.image).toImageMsg());

   
    
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
