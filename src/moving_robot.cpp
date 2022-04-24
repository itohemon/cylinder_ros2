#include <chrono>
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals;

class MovingRobot : public rclcpp::Node {
public:
  MovingRobot() : Node("moving_robot"), count_(0) {
    publisher_ =
      this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MovingRobot::timer_callback, this));
  }

private:
  void timer_callback() {
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = 0.0;
    message.angular.z = 0.0;
    RCLCPP_INFO(this->get_logger(), "Publishing: '%.2f' and '%.2f'",
      message.linear.x, message.angular.z);
    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char *argv[]) 
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MovingRobot>());
  rclcpp::shutdown();

  return 0;
}
