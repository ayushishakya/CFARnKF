#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char ** argv)
{

 ros::init(argc, argv, "input_image");
 ros::NodeHandle nh;
 image_transport::ImageTransport it(nh);

 image_transport::Publisher image_pub = it.advertise("input", 1);
 
 ros::Rate loop_rate(10);

 VideoCapture cap("/home/ayu/Videos/buoy.avi"); 
 Mat frame;
 
 while(nh.ok())
 {
if(!cap.isOpened()) cout<<"nai chal rha\n";

  if(cap.read(frame)) {
    cout<<"came\n";
   sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();

   image_pub.publish(msg);
   ROS_INFO("INPUT SENT\n");

   waitKey(1);
  }
  
  ros::spinOnce();
  loop_rate.sleep(); 
 }

 return 0;

}
