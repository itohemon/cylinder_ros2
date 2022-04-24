#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <cylinder_interfaces/action/cylinder.hpp>

/*** 名前空間を省略して利用できるように宣言 ***/
using Cylinder = cylinder_interfaces::action::Cylinder;
using GoalHandleCylinder = rclcpp_action::ClientGoalHandle<Cylinder>;

using namespace rclcpp_action;

namespace cylinder_ros2
{

class CylinderMove : public rclcpp::Node
{
public:
  CylinderMove(rclcpp::NodeOptions options = rclcpp::NodeOptions());
  ~CylinderMove();

private:
  void execute();

  /*** コールバック関数 ***/
  void onGoalResponseReceived(
    std::shared_future<GoalHandleCylinder::SharedPtr> future);
  void onFeedbackReceived(
    GoalHandleCylinder::SharedPtr goal_handle,
    const std::shared_ptr<const Cylinder::Feedback> feedback);
  void onResultReceived(const GoalHandleCylinder::WrappedResult &result);
  
  /*** メンバー変数 ***/
  Client<Cylinder>::SharedPtr action_client_;
  ResultCode result_code_;
};

/*** コンストラクタ ***/
CylinderMove::CylinderMove(rclcpp::NodeOptions options) : Node("cylinder_move", options)
{
  /*** アクションクライアントの初期化 ***/
  action_client_ = rclcpp_action::create_client<Cylinder>(this, "cylinder");

  /*** 別スレッドでアクションを呼び出し ***/
  std::thread{&CylinderMove::execute, this}.detach();
}

/*** デストラクタ ***/
CylinderMove::~CylinderMove()
{
}

/*** アクションを呼び出すための関数 ***/
void CylinderMove::execute()
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  /*** 目標地点のリストを生成 ***/
  std::array<std::array<double, 2>, 8> target_points =
  {
    std::array<double, 2>{ 0.5,  0.0}, std::array<double, 2>{ 0.0, -0.5},
    std::array<double, 2>{-0.5,  0.0}, std::array<double, 2>{ 0.0,  0.5},
    std::array<double, 2>{ 0.5,  0.5}, std::array<double, 2>{ 0.5, -0.5},
    std::array<double, 2>{-0.5, -0.5}, std::array<double, 2>{-0.5,  0.5}
  };

  /*** 前後左右斜め8方向に移動 ***/
  auto goal_msg = Cylinder::Goal();
  for (int i = 0; i < target_points.size(); i++) {
    /*** 目標地点の設定 ***/
    goal_msg.target.x = target_points[i][0];
    goal_msg.target.y = target_points[i][1];

    /*** コールバックの登録 ***/
    auto send_goal_options = Client<Cylinder>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&CylinderMove::onGoalResponseReceived, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&CylinderMove::onFeedbackReceived, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&CylinderMove::onResultReceived, this, _1);

    /*** アクションの呼び出し ***/
    result_code_ = ResultCode::UNKNOWN;
    auto result = action_client_->async_send_goal(goal_msg, send_goal_options);

    /*** アクションが終了するまで待機 ***/
    while (rclcpp::ok() && result_code_ == ResultCode::UNKNOWN) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

/*** アクション起動時に呼び出されるコールバック関数 ***/
void CylinderMove::onGoalResponseReceived(
  std::shared_future<GoalHandleCylinder::SharedPtr> future)
{
  auto goal_handle = future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(),
      "Goal accepted by server, waiting for result");
  }
}

/*** フィードバック受信時に呼び出されるコールバック関数 ***/
void CylinderMove::onFeedbackReceived(GoalHandleCylinder::SharedPtr goal_handle,
  const std::shared_ptr<const Cylinder::Feedback> feedback)
{
  RCLCPP_INFO(this->get_logger(), "Pose: (%.2f, %.2f)",
    feedback->pose.position.x, feedback->pose.position.y);
}

/*** アクション終了時に呼び出されるコールバック関数 ***/
void CylinderMove::onResultReceived(const GoalHandleCylinder::WrappedResult &result)
{
  result_code_ = result.code;
  switch(result.code) {
    /*** アクションが成功した場合 ***/
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Result message : %s",
        result.result->message.c_str());
      break;
    /*** アクションが何らかの原因で終了した場合 ***/
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal wa abortded");
      break;
    /*** アクションが中断された場合 ***/
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      break;
    /*** 上記以外の場合 ***/
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      break;
  }
}

}

/*** CylinderMoveクラスをコンポーネントとして登録 ***/
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(cylinder_ros2::CylinderMove)
