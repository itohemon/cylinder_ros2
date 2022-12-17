#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#define WHEEL_RAD 0.035   // ホイール半径
#define WHEEL_SEP 0.186   // トレッド

rclcpp::Time current_time;
rclcpp::Time last_time;

double x  = 0.0;
double y  = 0.0;
double th = 0.0;

using namespace std::chrono_literals;
using std::placeholders::_1;

class CylinderStatus : public rclcpp::Node
{
public:
  CylinderStatus() : Node("cylinder_status")
  {
    publish_tf_ = true;
//    this->declare_parameter("publish_tf");
    this->get_parameter("publish_tf", publish_tf_);

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "odom", 10);
    jointstate_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "joint_states", 10);

    wheelstate_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "wheel_state", 10, std::bind(&CylinderStatus::wheelStateCb, this, _1));

    odom_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    joint_state_l_ = "wheel_left_joint";
    joint_state_r_ = "wheel_right_joint";
  }
  
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
  
};

void CylinderStatus::wheelStateCb(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  auto odom_msg = nav_msgs::msg::Odometry();
  auto state_msg = sensor_msgs::msg::JointState();

  current_time = this->get_clock()->now();

  if (msg->data.size() < 1) return;

  double wL     = msg->data[0]; // 車軸回転角速度
  double jointL = msg->data[1]; // 車輪角度
  double wR     = msg->data[3]; // 車軸回転角速度
  double jointR = msg->data[4]; // 車輪角度

  //RCLCPP_INFO(this->get_logger(), "Subscribe wL: %lf  wR: %lf", wL, wR);

  // 車軸回転角速度から車体速度と角速度を算出
  double vx = WHEEL_RAD * (wR + wL) / 2;
  double vy = 0.0;
  double vth = WHEEL_RAD * (wR - wL) / WHEEL_SEP;
  // RCLCPP_INFO(this->get_logger(), "Subscribe vx: %lf  vth: %lf", vx, vth);
 
  double dt = current_time.seconds() - last_time.seconds();
  double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
  double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
  double delta_th = vth * dt;
  
  // [Reference]
  // http://wiki.ros.org/ja/navigation/Tutorials/RobotSetup/Odom
  x += delta_x;
  y += delta_y;
  th += delta_th;

  //since all odometry is 6DOF we'll need a quaternion created from yaw
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, th);

  //first, we'll publish the transform over tf
  odom_msg.header.stamp = current_time;
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_link";

  //set the position
  odom_msg.pose.pose.position.x = x;
  odom_msg.pose.pose.position.y = y;
  odom_msg.pose.pose.position.z = 0.0;

  odom_msg.pose.pose.orientation.x = q.x();
  odom_msg.pose.pose.orientation.y = q.y();
  odom_msg.pose.pose.orientation.z = q.z();
  odom_msg.pose.pose.orientation.w = q.w();

  //set the velocity
  odom_msg.twist.twist.linear.x = vx;
  odom_msg.twist.twist.linear.y = vy;
  odom_msg.twist.twist.angular.z = vth;

  odom_pub_->publish(odom_msg);

  // JointStatesを作ってパブリッシュする
  state_msg.header.stamp = current_time;
  state_msg.name.resize(2);
  state_msg.name[0] = joint_state_l_.c_str();
  state_msg.name[1] = joint_state_r_.c_str();
  state_msg.position.resize(2);
  state_msg.position[0] = jointL;
  state_msg.position[1] = jointR;
  
  jointstate_pub_->publish(state_msg);


  // odomフレーム(TF)をブロードキャストする
  geometry_msgs::msg::TransformStamped odom_tf;
  
  odom_tf.transform.translation.x = odom_msg.pose.pose.position.x;
  odom_tf.transform.translation.y = odom_msg.pose.pose.position.y;
  odom_tf.transform.translation.z = odom_msg.pose.pose.position.z;
  odom_tf.transform.rotation      = odom_msg.pose.pose.orientation;
  
  odom_tf.header.frame_id = odom_msg.header.frame_id;
  odom_tf.header.stamp    = odom_msg.header.stamp;
  odom_tf.child_frame_id  = odom_msg.child_frame_id;

  if (publish_tf_) {
    odom_tf_broadcaster_->sendTransform(odom_tf);
  }
  last_time = current_time;
}

int main(int argc,  char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CylinderStatus>());
  rclcpp::shutdown();

  return 0;
}
