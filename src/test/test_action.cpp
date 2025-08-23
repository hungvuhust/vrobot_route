#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "vrobot_route/action/move_to_pose.hpp"
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

using MoveToPose = vrobot_route::action::MoveToPose;
using namespace std::placeholders;

class MoveToPoseClient : public rclcpp::Node {
private:
  rclcpp_action::Client<MoveToPose>::SharedPtr action_client_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
                          pose_subscriber_;
  rclcpp::Node::SharedPtr node_;
  double                  x;
  double                  y;
  double                  theta;
  bool                    is_pose_received{false};

public:
  MoveToPoseClient() : Node("move_to_pose_client") {
    action_client_ =
      rclcpp_action::create_client<MoveToPose>(this, "move_to_pose");
    node_ = std::make_shared<rclcpp::Node>("mtp_node");
    pose_subscriber_ =
      node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/tracked_pose",
        10,
        std::bind(&MoveToPoseClient::pose_callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "Move To Pose Client đã khởi động");
  }

  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    x                = msg->pose.position.x;
    y                = msg->pose.position.y;
    theta            = msg->pose.orientation.z;
    is_pose_received = true;
  }

  void send_goal(const std::string &map_name, uint64_t target_node_id) {
    is_pose_received = false;
    // Chờ action server
    if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Action server không khả dụng!");
      return;
    }

    while (pose_subscriber_->get_publisher_count() == 0 or !is_pose_received) {
      RCLCPP_INFO(this->get_logger(), "Chờ publisher pose...");
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      rclcpp::spin_some(node_);
    }

    // Tạo goal
    auto goal_msg               = MoveToPose::Goal();
    goal_msg.map_name           = map_name;
    goal_msg.target_node_id     = target_node_id;
    goal_msg.current_pose.x     = x;
    goal_msg.current_pose.y     = y;
    goal_msg.current_pose.theta = theta;

    RCLCPP_INFO(this->get_logger(),
                "Gửi goal: map=%s, target_node=%lu, current_pose=(%f,%f,%f)",
                map_name.c_str(),
                target_node_id,
                x,
                y,
                theta);

    // Tạo send goal options
    auto send_goal_options =
      rclcpp_action::Client<MoveToPose>::SendGoalOptions();

    // Goal response callback
    send_goal_options.goal_response_callback =
      [this](const rclcpp_action::ClientGoalHandle<MoveToPose>::SharedPtr
               &goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(this->get_logger(), "Goal bị từ chối bởi server");
        } else {
          RCLCPP_INFO(this->get_logger(),
                      "Goal được chấp nhận, đang chờ kết quả...");
        }
      };

    // Feedback callback
    send_goal_options.feedback_callback =
      [this](rclcpp_action::ClientGoalHandle<MoveToPose>::SharedPtr,
             const std::shared_ptr<const MoveToPose::Feedback> feedback) {
        RCLCPP_INFO(this->get_logger(),
                    "Feedback: distance_remaining=%.2f, speed=%.2f",
                    feedback->distance_remaining,
                    feedback->speed);
      };

    // Result callback
    send_goal_options.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<MoveToPose>::WrappedResult
               &result) {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(),
                        "Thành công! Total distance: %.2f, Total time: %.2f",
                        result.result->total_distance,
                        result.result->total_time);
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(),
                         "Goal bị hủy bỏ: %s",
                         result.result->error_message.c_str());
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "Goal bị hủy");
            break;
          default:
            RCLCPP_ERROR(this->get_logger(), "Kết quả không xác định");
            break;
        }

        // Shutdown node after getting result
        rclcpp::shutdown();
      };

    // Gửi goal
    action_client_->async_send_goal(goal_msg, send_goal_options);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  if (argc != 3) {
    std::cout << "Usage: test_move_to_pose_client <map_name> <target_node_id> "
              << std::endl;
    std::cout << "Example: test_move_to_pose_client map1 123" << std::endl;
    return 1;
  }

  std::string map_name       = argv[1];
  uint64_t    target_node_id = std::stoull(argv[2]);

  auto client = std::make_shared<MoveToPoseClient>();

  // Gửi goal
  client->send_goal(map_name, target_node_id);

  // Spin để nhận callbacks
  rclcpp::spin(client);

  return 0;
}