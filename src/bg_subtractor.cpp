#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Int64MultiArray.h>

#include <iostream>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"

using namespace cv;
using namespace std;

ros::Publisher pub_n;

Mat background_subtractor(const sensor_msgs::ImageConstPtr& msg){
  
  ROS_INFO("IMAGE RECIEVED\n");
  cv_bridge::CvImagePtr cv_ptr;
  std_msgs::Int64MultiArray s;

  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  Mat image = cv_ptr->image;
  Mat hsv, grey;

  cvtColor(image, hsv, COLOR_BGR2HSV);

  for(int i=0; i<hsv.rows; i++)
    for(int j=0; j<hsv.cols; j++) 
        if(hsv.at<Vec3b>(i,j)[0] >80 && hsv.at<Vec3b>(i,j)[0] < 95)
            hsv.at<Vec3b>(i,j)[2] = 0;  

  cvtColor(hsv, grey, COLOR_BGR2GRAY);

// GaussianBlur( grey, grey, Size(9, 9), 2, 2 );
  vector<Vec3f> circles;
  int radius;
  Point center;
        
  HoughCircles( grey, circles, CV_HOUGH_GRADIENT, 3, 150, 200, 100, 50, 80 );

  for( size_t i = 0; i < circles.size(); i++ ) {
    center.x = cvRound(circles[i][0]);
    center.y = cvRound(circles[i][1]);
    radius = cvRound(circles[i][2]);
      // circle center
    circle( image, center, 3, Scalar(0,255,0), -1, 4, 0 );
      // circle outline
    circle( image, center, radius, Scalar(0,0,255), 3, 4, 0 );
  }
  
  s.data.push_back(center.x);
  s.data.push_back(center.y);
  s.data.push_back(radius);

  ROS_INFO("Values sent to topic-print\n");
  pub_n.publish(s);


  imshow("buoy", image);
  waitKey(1);
}

int main(int argc, char** argv){
 ros::init(argc, argv, "object_detect");
 ros::NodeHandle nh;
 image_transport::ImageTransport it(nh);

    pub_n = nh.advertise<std_msgs::Int64MultiArray>("state", 1);  
    
 ros::Rate loop_rate(10);

 image_transport::Subscriber image_sub_ = it.subscribe("input", 1, background_subtractor);

 ros::spin();
 return 0;
}