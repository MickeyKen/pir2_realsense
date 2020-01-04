#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
float distance = 0.0;
// void imageCallback(const sensor_msgs::Image::ConstPtr& image_msg)
// {
//   float z_mean = 0.0;
//   // Solve all of perception here...
//   const std::string& enc = image_msg->encoding;
//
//   if (enc.compare("16UC1") == 0)
//
//   depth_img = cv_bridge::toCvShare(image_msg)->image;
//
//   else if (enc.compare("32FC1") == 0)
//
//   depthImageFloatTo16bit(cv_bridge::toCvShare(image_msg)->image, depth_img);
//
//   // get raw z value (in mm)
//
//   uint16_t z_raw = depth_img.at<uint16_t>(0, 0);
//
//   // z [meters]
//
//   z_mean = z_raw * 0.001;
//  }
 void imageCallback(const sensor_msgs::ImageConstPtr& msg)
 {
   cv_bridge::CvImagePtr cv_ptr;
   try
   {
     cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
   }
   catch (cv_bridge::Exception& e)
   {
     ROS_ERROR("cv_bridge exception: %s", e.what());
     // return;
   }

   distance = 0.001*cv_ptr->image.at<u_int16_t>(0, 0);
   std::cout<<distance<<std::endl;
 }


 int main(int argc, char **argv)
 {
   ros::init(argc, argv, "image_listener");
   ros::NodeHandle nh;
   ros::Subscriber sub = nh.subscribe("camera/image", 1000, imageCallback);
   ros::spin();
 }
