
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/empty.hpp"
#include "vrobot_route/action/move_to_pose.hpp"
#include "vrobot_route/srv/move_to_pose.hpp"
#include "vrobot_route/node_thread.hpp"
#include <atomic>
#include <chrono>
#include <functional>
#include <future>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <mutex>
#include <rclcpp/client.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/service.hpp>
#include <string>
#include <thread>

using namespace std::placeholders;

constexpr char kActionMoveName[]    = "/move_to_pose";
constexpr char kServiceMoveName[]   = "/vrobot/move/go";
constexpr char kServiceCancelName[] = "/vrobot/move/cancel";

using MoveToPoseAct  = vrobot_route::action::MoveToPose;
using GoalHandleMove = rclcpp_action::ClientGoalHandle<MoveToPoseAct>;
using Empty          = std_srvs::srv::Empty;
using MoveToPoseSrv  = vrobot_route::srv::MoveToPose;

// Action client wrapper via service
class ServiceActionMove : public rclcpp::Node {
public:
  ServiceActionMove() : Node("service_action_move") {
    // Callback group
    callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Action client
    action_client_ =
      rclcpp_action::create_client<MoveToPoseAct>(this, kActionMoveName);
    node_    = rclcpp::Node::make_shared("service_move");
    // Service
    service_ = node_->create_service<MoveToPoseSrv>(
      kServiceMoveName,
      std::bind(&ServiceActionMove::service_callback, this, _1, _2));

    pose_subscriber_ =
      this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/tracked_pose",
        10,
        std::bind(&ServiceActionMove::pose_callback, this, _1));

    service_cancel_ = this->create_service<Empty>(
      kServiceCancelName,
      [this](const std::shared_ptr<Empty::Request> /*request*/,
             std::shared_ptr<Empty::Response> /*response*/) {
        if (is_processing_.load()) {
          action_client_->async_cancel_all_goals();
        }
      });

    service_thread_ = std::make_unique<vrobot_route::NodeThread>(node_);
  }

  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    x_                = msg->pose.position.x;
    y_                = msg->pose.position.y;
    theta_            = msg->pose.orientation.z;
    is_pose_received_ = true;
  }

  void service_callback(const std::shared_ptr<MoveToPoseSrv::Request> request,
                        std::shared_ptr<MoveToPoseSrv::Response> response) {
    if (is_processing_.load()) {
      response->success       = false;
      response->error_message = "Action is processing";
      return;
    }

    if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
      response->success       = false;
      response->error_message = "Action server not found";
      return;
    }

    is_pose_received_ = false;

    while (pose_subscriber_->get_publisher_count() == 0 or !is_pose_received_) {
      RCLCPP_INFO(this->get_logger(), "Chờ publisher pose...");
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    auto goal_msg               = MoveToPoseAct::Goal();
    goal_msg.map_name           = request->map_name;
    goal_msg.target_node_id     = request->target_node_id;
    goal_msg.current_pose.x     = x_;
    goal_msg.current_pose.y     = y_;
    goal_msg.current_pose.theta = theta_;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options =
      rclcpp_action::Client<MoveToPoseAct>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&ServiceActionMove::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&ServiceActionMove::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&ServiceActionMove::result_callback, this, _1);

    auto goal_handle_future =
      action_client_->async_send_goal(goal_msg, send_goal_options);

    is_processing_.store(true);

    while (is_processing_.load() && rclcpp::ok()) {  // Wait for the result
      //   RCLCPP_INFO(this->get_logger(), "Waiting for goal to finish");
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    response->success       = success_;
    response->error_message = error_message_;
  }

private:
  void goal_response_callback(GoalHandleMove::SharedPtr future) {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
      is_processing_.store(false);

    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandleMove::SharedPtr,
    const std::shared_ptr<const MoveToPoseAct::Feedback> /*feedback*/) {
    // RCLCPP_INFO(this->get_logger(), "Feedback received: Vel: %f, Dist: %f",
    //             feedback->speed, feedback->distance_to_goal);
  }

  void result_callback(
    const rclcpp_action::ClientGoalHandle<MoveToPoseAct>::WrappedResult
      &result) {
    RCLCPP_INFO(this->get_logger(),
                "Action server result: %s",
                result.result->error_message.c_str());

    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Reached");
        success_       = true;
        error_message_ = "Action completed";
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        success_       = false;
        error_message_ = "Goal was aborted";
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        success_       = false;
        error_message_ = "Goal was canceled";
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        success_       = false;
        error_message_ = "Unknown result code";
        break;
    }

    is_processing_.store(false);
  }

private:
  //   Action client
  rclcpp_action::Client<MoveToPoseAct>::SharedPtr action_client_;

  //   Service
  rclcpp::Service<MoveToPoseSrv>::SharedPtr service_;
  rclcpp::Service<Empty>::SharedPtr         service_cancel_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
              pose_subscriber_;
  double      x_{0.0};
  double      y_{0.0};
  double      theta_{0.0};
  bool        is_pose_received_{false};
  //   result
  bool        success_{false};
  std::string error_message_{""};

  // atomic
  std::atomic<bool> is_processing_{false};

  // callback group
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  //   Node thread
  std::unique_ptr<vrobot_route::NodeThread> service_thread_;
  rclcpp::Node::SharedPtr                   node_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServiceActionMove>());
  rclcpp::shutdown();
  return 0;
}