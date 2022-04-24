#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class Imu2Pose : public rclcpp::Node {
public:
  Imu2Pose(
    const std::string& name_space = "",
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
    );
private:
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _sub;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _pub;
  void _topic_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

};

Imu2Pose::Imu2Pose(const std::string& name_space, const rclcpp::NodeOptions& options)
: Node("imu2pose", name_space, options)
{
  _sub = this->create_subscription<sensor_msgs::msg::Imu>(
    "/bno055/imu",
    rclcpp::QoS(10),
    std::bind(&Imu2Pose::_topic_callback, this, std::placeholders::_1)
    );

  _pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/imu_pose", 10);
}

void Imu2Pose::_topic_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  auto ps_msg = std::make_shared<geometry_msgs::msg::PoseStamped>();

  ps_msg->header = msg->header;
  ps_msg->pose.orientation = msg->orientation;
  _pub->publish(*ps_msg);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Imu2Pose>());
  rclcpp::shutdown();

  return 0;
}
