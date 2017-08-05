#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";

/// Global variables

int maxCorners = 23;
int maxTrackbar = 100;

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

	void goodFeaturesToTrack_Demo( int, void* ,cv_bridge::CvImage  gray, std::vector<cv::Point2f> corners)  ///[1]
{
  if( maxCorners < 1 ) { 
 		maxCorners = 1; 
  }

  /// Parameters for Shi-Tomasi algorithm
  //std::vector<cv::Point2i> corners;       ///[2]
  double qualityLevel = 0.01;
  double minDistance = 10;
  int blockSize = 3;
  bool useHarrisDetector = false;
  double k = 0.04;

  /// Copy the source image
  

  /// Apply corner detection
  goodFeaturesToTrack( gray.image,
               corners,
               maxCorners,
               qualityLevel,
               minDistance,
               cv::Mat(),                    ///[3]
               blockSize,
               useHarrisDetector,
               k );


  /// Draw corners detected
 // cout<<"** Number of corners detected: "<<corners.size()<<endl;
 // int r = 4;
  //for( size_t i = 0; i < corners.size(); i++ )
    // { cv::circle(gray.image, corners[i], 10, CV_RGB(255,0,0)); }  ///[4]

  
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
     
      cv_bridge::CvImage preview, nextview;
      std::vector<cv::Point2f>  points;
      std::vector<cv::Point2f>  newpoints;
                        
      cv::cvtColor(cv_ptr->image, nextview.image, CV_BGR2GRAY);  
      
     // goodFeaturesToTrack_Demo( 0, 0, nextview );
         
      //2color white/black
      //cv::threshold(gray.image, gray.image, 127, 255, cv::THRESH_BINARY); 

////////////////////////////////////////////////////////			
			if(&preview.image != 0){	
			
				//std::vector<cv::KeyPoint>  keypoints;
  	   //	std::vector<cv::Point2f>   prepoints;
  	    	
  	 	  //cv::GoodFeaturesToTrackDetector detector(100, 0.05, 3);
  	 	  //detector.detect(preview.image, keypoints);
  	  	  
  	 	  //for(std::vector<cv::KeyPoint>::iterator itk = keypoints.begin(); itk != keypoints.end(); ++itk){
  	 	  	//prepoints.push_back(itk->pt);
  	 	  //}
  	 	  
				std::vector<unsigned char>  status;
				std::vector<float>   errors;
				
				
			  calcOpticalFlowPyrLK(preview.image, nextview.image, points, newpoints, status, errors, cv::Size(21,21), 3, cvTermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.05), 0, 0.001);
			  
			  			  
				}
  	    
			cv::cvtColor(nextview.image, preview.image, CV_BGR2GRAY);
			goodFeaturesToTrack_Demo( 0, 0, preview, points );
			
			
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
