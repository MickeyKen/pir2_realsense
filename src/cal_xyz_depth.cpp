#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <people_msgs/HeadPoseStamped.h>
#include <people_msgs/HeadPose.h>
#include <geometry_msgs/Twist.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

float distance = 0.0;
int u = 0;
int v = 0;

int image_offset = 50;
float dis_offset = 0.2;

ros::Publisher cmd_pub;
int number = 0;
float HZ = 1;
geometry_msgs::Twist cmd_msg;

using namespace message_filters;

void syncMsgsCB(const sensor_msgs::ImageConstPtr &img_msg, const people_msgs::HeadPoseStampedConstPtr &pose_msg){

   u = pose_msg->headposes[0].roi.x_offset + (pose_msg->headposes[0].roi.width/2);
   std::cout<<u<<std::endl;

   if (u > (320-image_offset) && u < (320+image_offset)) {
     cmd_msg.angular.z = 0.0;

     cv_bridge::CvImagePtr cv_ptr;
     try
     {
       cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::TYPE_16UC1);
     }
     catch (cv_bridge::Exception& e)
     {
       ROS_ERROR("cv_bridge exception: %s", e.what());
       // return;
     }

     distance = 0.001*cv_ptr->image.at<u_int16_t>(u, 240);
     std::cout<<distance<<std::endl;

     if (distance > (1.2 - dis_offset) && distance < (1.2 + dis_offset)) {
       cmd_msg.linear.x = 0.0;

     } else if (distance < (1.2 - dis_offset) && distance > 0.0) {
       cmd_msg.linear.x = -0.15;

     } else if (distance > (1.2 + dis_offset && distance < 2.5)) {
       cmd_msg.linear.x = 0.15;

     } else {
       cmd_msg.linear.x = 0.0;
     }

   } else if (u < (320-image_offset)) {
     cmd_msg.linear.x = 0.0;
     if (u > (320-image_offset-100)) {
       cmd_msg.angular.z = 0.15;
     } else {
       cmd_msg.angular.z = 0.3;
     }

   } else if (u > (320+image_offset)) {
     cmd_msg.linear.x = 0.0;
     if (u < (320+image_offset+100)) {
       cmd_msg.angular.z = -0.15;
     } else {
       cmd_msg.angular.z = -0.3;
     }
   } else {
     cmd_msg.linear.x = 0.0;
     cmd_msg.angular.z = 0.0;
   }

 }


 int main(int argc, char **argv)
 {
   ros::init(argc, argv, "image_listener");
   ros::NodeHandle nh;
   message_filters::Subscriber<sensor_msgs::Image> odom_sub(nh, "camera/depth/image_rect_raw", 1000);
   message_filters::Subscriber<people_msgs::HeadPoseStamped> imu_sub(nh, "/ros_openvino_toolkit/headposes_estimation", 1000);

   typedef sync_policies::ApproximateTime<sensor_msgs::Image, people_msgs::HeadPoseStamped> MySyncPolicy;
   message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), odom_sub, imu_sub);
   sync.registerCallback(boost::bind(&syncMsgsCB, _1, _2));

   cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
   cmd_msg = geometry_msgs::Twist();

   if (HZ > 0)
   {
     ros::Rate loop_rate(10);
     while (ros::ok())
     {
       cmd_pub.publish(cmd_msg);
       ros::spinOnce();
       loop_rate.sleep();
     }
   }
   else
     ros::spin();
   return 0;

 }
