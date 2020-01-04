#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <people_msgs/HeadPoseStamped.h>
#include <people_msgs/HeadPose.h>
#include <cv_bridge/cv_bridge.h>

using namespace sensor_msgs;
using namespace message_filters;

void callback(const ImageConstPtr& image_msg, const HeadPoseStampedConstPtr& pose_msg)
{
  // Solve all of perception here...
  const std::string& enc = image_msg->encoding;

  if (enc.compare("16UC1") == 0)

  depth_img = cv_bridge::toCvShare(image_msg)->image;

  else if (enc.compare("32FC1") == 0)

  depthImageFloatTo16bit(cv_bridge::toCvShare(image_msg)->image, depth_img);

  // get raw z value (in mm)

  uint16_t z_raw = depth_img.at<uint16_t>(0, 0);

  // z [meters]

  z_mean = z_raw * 0.001;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vision_node");

  ros::NodeHandle nh;
  message_filters::Subscriber<Image> image_sub(nh, "/camera/aligned_depth_to_color/image_raw", 1);
  message_filters::Subscriber<HeadPoseStamped> pose_sub(nh, "/ros_openvino_toolkit/headposes_estimation", 1);

  typedef sync_policies::ApproximateTime<Image, HeadPoseStamped> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, pose_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();

  return 0;
}
