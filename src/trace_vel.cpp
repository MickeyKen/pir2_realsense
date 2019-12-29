#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

ros::Publisher cmd_pub;

int number = 0;
float HZ = 1;

geometry_msgs::Twist cmd_msg;

void Callback(const geometry_msgs::Twist::ConstPtr& msg)
{
  // filtered_pub.publish(msg);
  cmd_msg = *msg;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pub_trace_vel");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");

  cmd_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
  ros::Subscriber pose_sub = n.subscribe("/pir2/trace_pose", 1000, Callback);
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
