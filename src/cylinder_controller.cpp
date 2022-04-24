#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <cylinder_interfaces/action/cylinder.hpp>
#include <cmath>

/*** 名前空間を省略して利用できるように宣言 ***/
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Twist;
using nav_msgs::msg::Odometry;
using std_msgs::msg::Float64;
using Cylinder = cylinder_interfaces::action::Cylinder;

using namespace rclcpp_action;

/*** typedefを用いた型の再定義 ***/
typedef std::shared_ptr<ServerGoalHandle<Cylinder>> GoalHandleCylinderPtr;
typedef std::shared_ptr<const Cylinder::Goal> GoalPtr;

namespace cylinder_ros2
{

class CylinderController : public rclcpp::Node
{
public:
  CylinderController(rclcpp::NodeOptions = rclcpp::NodeOptions());
  ~CylinderController();

private:
  void execute(const GoalHandleCylinderPtr goal_handle);
  /*** コールバック関数 ***/
  void onPoseSubscribed(const Odometry::SharedPtr pose);
  void onDistanceSubscribed(const Float64::SharedPtr distance);
  GoalResponse onGoalSet(const GoalUUID &uuid, GoalPtr goal);
  CancelResponse onActionCanceled(const GoalHandleCylinderPtr goal_hendle);
  void onActionAccepted(const GoalHandleCylinderPtr goal_handle);

  /*** メンバー変数 ***/
  rclcpp::Publisher<Twist>::SharedPtr pub_vel_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<Float64>::SharedPtr sub_distance_;
  rclcpp_action::Server<Cylinder>::SharedPtr action_server_;
  double distance_ = .0;
  double vel_;
  Pose pose_;
};

/*** コンストラクタ ***/
CylinderController::CylinderController(rclcpp::NodeOptions options)
: Node("cylinder_controller", options)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  /*** パラメータの読み込み ***/
  vel_ = this->declare_parameter<double>("vel", 0.1);
  /*** パブリッシャの初期化  ***/
  pub_vel_ = this->create_publisher<Twist>("/cmd_vel", 10);
  /*** サブスクライバの初期化 ***/
  sub_odom_ = this->create_subscription<Odometry>(
    "/odom", 10,
    std::bind(&CylinderController::onPoseSubscribed, this, _1)
    );
  sub_distance_ = this->create_subscription<Float64>(
    "/Cylinder/distance", 10,
    std::bind(&CylinderController::onDistanceSubscribed, this, _1)
    );
  /*** アクションサーバの初期化 ***/
  action_server_ = create_server<Cylinder>(
    this, "Cylinder",
    std::bind(&CylinderController::onGoalSet, this, _1, _2),
    std::bind(&CylinderController::onActionCanceled, this, _1),
    std::bind(&CylinderController::onActionAccepted, this, _1)
    );
}

/*** デストラクタ ***/
CylinderController::~CylinderController()
{
}

/*** アクションの実体 ***/
void CylinderController::execute(const GoalHandleCylinderPtr goal_handle)
{
  double old_distance = .0;
  Twist stop_vel, target_vel;
  auto goal = goal_handle->get_goal();

  /*** 目標地点の方向からVx,Vyを決定 ***/
  if (goal->target.x == 0 && goal->target.y == 0) {
    target_vel = stop_vel;
  } else {
    double target_dir = atan2(goal->target.y, goal->target.x);
    target_vel.linear.x = vel_ * cos(target_dir);
    target_vel.linear.y = vel_ * sin(target_dir);
  }

  /*** 目標地点までの距離を算出 ***/
  double target_distance =
    sqrt(goal->target.x * goal->target.x + goal->target.y * goal->target.y);
  old_distance = distance_;

  /*** ループ周期の設定 ***/
  rclcpp::Rate loop(10);

  /*** フィードバックとリザルトの宣言 ***/
  auto feedback = std::make_shared<Cylinder::Feedback>();
  auto result = std::make_shared<Cylinder::Result>();

  /*** 目標地点までの移動制御 ***/
  while (rclcpp::ok()) {
    /*** アクションがキャンセルされた場合 ***/
    if (goal_handle->is_canceling()) {
      /*** 結果のメッセージを送信 ***/
      result->message = "Canceled";
      goal_handle->canceled(result);
      break;
    }
    /*** 目標地点まで到達した場合 ***/
    else if ((distance_ - old_distance) >= target_distance) {
      /*** 結果のメッセージを送信 ***/
      result->message = "Succeeded";
      goal_handle->succeed(result);
      pub_vel_->publish(stop_vel);
      break;
    }
    /*** 目標地点まで到達していない場合 ***/
    else {
      /*** 現在の座標を送信 ***/
      feedback->pose = pose_;
      goal_handle->publish_feedback(feedback);
      pub_vel_->publish(target_vel);
    }
    loop.sleep();
  }
}

/*** poseを受信した際に呼び出されるコールバック関数 ***/
void CylinderController::onPoseSubscribed(const Odometry::SharedPtr pose)
{
  pose_ = pose->pose.pose;
}

/*** distanceを受信した際に呼び出されるコールバック関数 ***/
void CylinderController::onDistanceSubscribed(const Float64::SharedPtr distance)
{
  distance_ = distance->data;
}

/*** ゴールが設定されたときに呼び出されるコールバック関数 ***/
GoalResponse CylinderController::onGoalSet(const GoalUUID & uuid, GoalPtr goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  return GoalResponse::ACCEPT_AND_EXECUTE;
}

/*** アクションがキャンセルされたときに呼び出されるコールバック関数 ***/
CancelResponse CylinderController::onActionCanceled(const GoalHandleCylinderPtr goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  return CancelResponse::ACCEPT;
}

/*** アクションが受理されたときに呼び出されるコールバック関数 ***/
void CylinderController::onActionAccepted(const GoalHandleCylinderPtr goal_handle)
{
  using namespace std::placeholders;
  RCLCPP_INFO(this->get_logger(), "Accepted action");
  std::thread{
    std::bind(&CylinderController::execute, this, _1),
      goal_handle
      }.detach();
}

}

/*** CylindeControllerクラスをコンポーネントとして登録 ***/
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(cylinder_ros2::CylinderController)
