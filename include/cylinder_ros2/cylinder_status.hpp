#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class CylinderStatus : public rclcpp::Node
{
public:
  CylinderStatus();
  
private:
  void wheelStateCb(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointstate_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr wheelstate_sub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> odom_tf_broadcaster_;

  bool publish_tf_;
  std::string joint_state_l_;
  std::string joint_state_r_;

  rclcpp::Time current_time_;
  rclcpp::Time last_time_;
  
  double x_  = 0.0;
  double y_  = 0.0;
  double th_ = 0.0;
};
