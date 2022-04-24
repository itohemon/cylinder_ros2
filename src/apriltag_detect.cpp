#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>

#include <apriltag_msgs/msg/april_tag_detection_array.hpp>

using sensor_msgs::msg::Image;
using apriltag_msgs::msg::AprilTagDetectionArray;
using message_filters::sync_policies::ApproximateTime;
using message_filters::Synchronizer;

using std::placeholders::_1;
using std::placeholders::_2;

rclcpp::Node::SharedPtr g_node = nullptr;

cv_bridge::CvImageConstPtr cv_img;

void onImageAprilTagSubscribed(Image::ConstSharedPtr image_msg,
  AprilTagDetectionArray::ConstSharedPtr tag_msg)
{

  cv_img = cv_bridge::toCvShare(image_msg, "bgr8");
  cv::imshow("Image", cv_img->image);
  cv::waitKey(1);

  int num = tag_msg->detections.size();

  auto cv_addTagImg = cv_bridge::toCvCopy(image_msg, "bgr8");
  for (int i = 0; i < num; i++) {
    RCLCPP_INFO(g_node->get_logger(), "AprilTagDetect[%d] %s %d",
      i,
      tag_msg->detections[i].family.c_str(),
      tag_msg->detections[i].id );
    cv::Point p0(tag_msg->detections[i].corners[0].x, tag_msg->detections[i].corners[0].y);
    cv::Point p1(tag_msg->detections[i].corners[1].x, tag_msg->detections[i].corners[1].y);
    cv::Point p2(tag_msg->detections[i].corners[2].x, tag_msg->detections[i].corners[2].y);
    cv::Point p3(tag_msg->detections[i].corners[3].x, tag_msg->detections[i].corners[3].y);
    cv::line(cv_addTagImg->image, p0, p1, cv::Scalar(0, 0, 255), 2);
    cv::line(cv_addTagImg->image, p1, p2, cv::Scalar(0, 0, 255), 2);
    cv::line(cv_addTagImg->image, p2, p3, cv::Scalar(0, 0, 255), 2);
    cv::line(cv_addTagImg->image, p3, p0, cv::Scalar(0, 0, 255), 2);
  }
  cv::imshow("aprilTagImage", cv_addTagImg->image);
  cv::waitKey(1);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  g_node = rclcpp::Node::make_shared("apriltag_image_subscriber");
  typedef ApproximateTime<Image, AprilTagDetectionArray> SyncPolicy;
  std::shared_ptr<message_filters::Subscriber<Image>> sub1;
  std::shared_ptr<message_filters::Subscriber<AprilTagDetectionArray>> sub2;
  std::shared_ptr<Synchronizer<SyncPolicy>> sync_msgs_;

  sub1.reset(new message_filters::Subscriber<Image>(g_node, "image_raw"));
  sub2.reset(new message_filters::Subscriber<AprilTagDetectionArray>(g_node, "apriltag/detections"));

  sync_msgs_.reset(new Synchronizer<SyncPolicy>(SyncPolicy(10), *sub1, *sub2));
  sync_msgs_->registerCallback(std::bind(onImageAprilTagSubscribed, _1, _2));

  /*
  auto sub_image = g_node->create_subscription<Image>("image_raw", 10, onImageSubscribed);
  auto sub_apriltag = g_node->create_subscription<AprilTagDetectionArray>("apriltag/detections", 10, onAprilTagSubscribed);
  */

  rclcpp::spin(g_node);

  g_node = nullptr;
  rclcpp::shutdown();

  return 0;
}
