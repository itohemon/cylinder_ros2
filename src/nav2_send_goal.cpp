#include <memory>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/time.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class Nav2Client : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;

  explicit Nav2Client(): Node("nav2_send_goal")
  {
    this->client_ptr_ 
      = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
  }

  void sendGoal(void) {
    while (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_INFO(get_logger(), "Waiting for action server...");
    }

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.stamp = this->now();
    goal_msg.pose.header.frame_id = "map";

    goal_msg.pose.pose.position.x = 2.0;
    goal_msg.pose.pose.position.y = 0.0;
    goal_msg.pose.pose.position.z = 0.0;
    goal_msg.pose.pose.orientation.x = 0.0;
    goal_msg.pose.pose.orientation.y = 0.0;
    goal_msg.pose.pose.orientation.w = 1.0;
    goal_msg.pose.pose.orientation.z = 0.0;
    
    auto send_goal_options
      = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    send_goal_options.feedback_callback
      = std::bind(&Nav2Client::feedbackCallback, this, _1, _2);
    send_goal_options.result_callback
      = std::bind(&Nav2Client::resultCallback, this, _1);

    client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

  void feedbackCallback(
    GoalHandleNavigateToPose::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
  {
    RCLCPP_INFO(get_logger(), "Distance remaining = %f", feedback->distance_remaining);
  }

  void resultCallback(
    const GoalHandleNavigateToPose::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(get_logger(), "Success!");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(get_logger(), "Goal was canceded");
        return;
      default:
        RCLCPP_ERROR(get_logger(), "Unknown result code");
        return;
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Nav2Client>();
  node->sendGoal();
  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
