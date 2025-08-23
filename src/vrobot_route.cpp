#include "rclcpp/rclcpp.hpp"
#include <vrobot_route/graph_pose.hpp>
#include <nav_msgs/msg/path.hpp>
#include <vrobot_route/vrobot_route.hpp>
#include <vrobot_route/benzier.hpp>

namespace vrobot_route {

constexpr char *kActionMoveToPoseName  = "/move_to_pose";
constexpr char *kActionMoveToPoseTopic = "/vrobot_route/path";
constexpr char *kGraphVisTopic         = "/graph_vis";
constexpr char *kActionVFollowPathName = "/v_follow_path";
constexpr char *kFollowPathTopic       = "/vrobot_route/follow_path";

VrobotRoute::VrobotRoute() : rclcpp::Node("vrobot_route") {
  init_db();
  init_publishers();
  init_action_server();
}

VrobotRoute::~VrobotRoute() {
}

void VrobotRoute::init_db() {
  db_manager_ = std::make_shared<vrobot_route::DbManager>(connection_info_);
}

void VrobotRoute::init_publishers() {
  graph_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    kGraphVisTopic, rclcpp::QoS(10).transient_local());

  path_publisher_ =
    create_publisher<nav_msgs::msg::Path>(kActionMoveToPoseTopic,
                                          rclcpp::QoS(10).transient_local());
  vpath_publisher_ = create_publisher<vrobot_local_planner::msg::Path>(
    kFollowPathTopic, rclcpp::QoS(10).transient_local());
}

void VrobotRoute::init_action_server() {
  // Create v follow path action client
  v_follow_path_client_ =
    rclcpp_action::create_client<vrobot_local_planner::action::VFollowPath>(
      this, kActionVFollowPathName);
  // Create move to pose action server
  action_server_ =
    std::make_unique<ActionServer>(this,
                                   kActionMoveToPoseName,
                                   std::bind(&VrobotRoute::execute_move_to_pose,
                                             this),
                                   nullptr,
                                   std::chrono::milliseconds(500),
                                   true);
  action_server_->activate();
}

void VrobotRoute::send_follow_path_goal(
  const std::vector<vrobot_route::v_edge_t> &path,
  bool                                       is_last_goal) {
  // Convert path to nav_msgs::msg::Path
  auto vpath    = graph_->pathSegmentsToVPath(path, "map", this->now(), 0.01);
  auto nav_path = graph_->toPath(vpath);
  vpath_publisher_->publish(vpath);
  publish_path(nav_path);
  is_goal_completed_ = false;
  // Send goal to follow path action server
  auto goal          = vrobot_local_planner::action::VFollowPath::Goal();
  goal.path          = vpath;
  if (!is_last_goal) {
    goal.goal_checker_id = "simple_goal_checker";
  } else {
    goal.goal_checker_id = "simple_goal_checker";
  }

  auto send_goal_options = rclcpp_action::Client<
    vrobot_local_planner::action::VFollowPath>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    [this](rclcpp_action::ClientGoalHandle<
           vrobot_local_planner::action::VFollowPath>::SharedPtr goal_handle) {
      RCLCPP_INFO(get_logger(), "Goal accepted");
    };
  send_goal_options.feedback_callback =
    [this](
      rclcpp_action::ClientGoalHandle<
        vrobot_local_planner::action::VFollowPath>::SharedPtr goal_handle,
      const std::shared_ptr<
        const vrobot_local_planner::action::VFollowPath::Feedback> feedback) {
      RCLCPP_DEBUG(get_logger(),
                   "Feedback: %f, %f",
                   feedback->speed,
                   feedback->distance_to_goal);

      auto feedback_msg   = std::make_shared<typename Action::Feedback>();
      feedback_msg->speed = feedback->speed;
      feedback_msg->distance_remaining = feedback->distance_to_goal;
      action_server_->publish_feedback(feedback_msg);
    };
  send_goal_options.result_callback =
    [this](const rclcpp_action::ClientGoalHandle<
           vrobot_local_planner::action::VFollowPath>::WrappedResult
             &follow_result) {
      RCLCPP_INFO(get_logger(), "Goal completed");

      switch (follow_result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED: {
          RCLCPP_INFO(get_logger(), "Goal completed");
          is_goal_completed_ = true;
          cv_follow_path_goal_.notify_all();
          break;
        }

        case rclcpp_action::ResultCode::ABORTED: {
          RCLCPP_ERROR(get_logger(), "Goal aborted");
          is_goal_completed_ = true;
          cv_follow_path_goal_.notify_all();
        } break;

        case rclcpp_action::ResultCode::CANCELED: {
          RCLCPP_WARN(get_logger(), "Goal canceled");
          is_goal_completed_ = true;
          cv_follow_path_goal_.notify_all();
        } break;

        default: {
          RCLCPP_ERROR(get_logger(), "Goal failed");
          is_goal_completed_ = true;
          cv_follow_path_goal_.notify_all();
        } break;
      }
    };

  v_follow_path_client_->async_send_goal(goal, send_goal_options);
  std::unique_lock<std::mutex> lock(mtx_follow_path_goal_);
  cv_follow_path_goal_.wait(lock, [this] { return is_goal_completed_; });
}

void VrobotRoute::execute_move_to_pose() {
  RCLCPP_INFO(get_logger(), "Executing move to pose");
  try {
    auto goal = action_server_->get_current_goal();
    RCLCPP_INFO(get_logger(), "Goal: %ld", goal->target_node_id);
    RCLCPP_INFO(get_logger(),
                "Goal: %f, %f, %f",
                goal->current_pose.x,
                goal->current_pose.y,
                goal->current_pose.theta);

    auto result =
      getPath(Eigen::Vector2d(goal->current_pose.x, goal->current_pose.y),
              goal->target_node_id,
              goal->map_name);

    if (result.pathSegments.empty()) {
      RCLCPP_ERROR(get_logger(), "No path found");
      std::shared_ptr<Action::Result> result =
        std::make_shared<Action::Result>();
      action_server_->terminate_current(result);
      return;
    }

    auto subdivided_paths = subdivide_sharp_turns(result.pathSegments);

    // Check if follow path action is available
    if (!v_follow_path_client_->wait_for_action_server(
          std::chrono::seconds(5))) {
      throw std::runtime_error("Follow path action server is not available");
    }

    // Send follow path goal for each subpath
    for (size_t i = 0; i < subdivided_paths.size(); ++i) {
      send_follow_path_goal(subdivided_paths[i],
                            i == subdivided_paths.size() - 1);
    }

  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    action_server_->terminate_current(result);
    return;
  }
  action_server_->succeeded_current();
}

bool VrobotRoute::update_graph(std::string map_name) {
  try {
    if (!db_manager_) {
      RCLCPP_ERROR(get_logger(), "Database manager not initialized");
      return false;
    }

    map_name_ = map_name;

    auto map           = db_manager_->getMap(map_name_);
    auto nodes         = db_manager_->getNodes(map_name_);
    auto straightlinks = db_manager_->getStraightlinks(map_name_);
    auto curvelinks    = db_manager_->getCurvelinks(map_name_);

    vnodes_.clear();
    vedges_.clear();

    std::unordered_map<int, vrobot_route::v_node_t> vnodes_map;
    RCLCPP_INFO(get_logger(), "Nodes: %ld", nodes.size());
    for (const auto &node : nodes) {
      vrobot_route::v_node_t vnode;
      vnode.id_    = node.getValueOfId();
      vnode.pose_  = Eigen::Vector2d(node.getValueOfX(), node.getValueOfY());
      vnode.theta_ = node.getValueOfTheta();

      vnodes_.push_back(vnode);
      vnodes_map[vnode.id_] = vnode;
    }

    RCLCPP_INFO(get_logger(), "Straightlinks: %ld", straightlinks.size());
    for (const auto &straightlink : straightlinks) {
      vrobot_route::v_edge_t vedge;
      vedge.id_         = straightlink.getValueOfIdStraightLink();
      vedge.start_node_ = vnodes_map[straightlink.getValueOfIdStart()];
      vedge.end_node_   = vnodes_map[straightlink.getValueOfIdEnd()];
      vedge.type_       = vrobot_route::v_link_type_t::STRAIGHT;
      vedge.length_     = (vnodes_map[straightlink.getValueOfIdStart()].pose_ -
                       vnodes_map[straightlink.getValueOfIdEnd()].pose_)
                        .norm();
      vedge.width_ = 0.5;
      vedge.max_vel_ =
        straightlink.getMaxVelocity() ? *straightlink.getMaxVelocity() : 2.0;

      vedges_.push_back(vedge);
    }
    RCLCPP_INFO(get_logger(), "Curvelinks: %ld", curvelinks.size());
    for (const auto &curvelink : curvelinks) {
      vrobot_route::v_edge_t vedge;
      vedge.id_         = curvelink.getValueOfIdCurveLink();
      vedge.start_node_ = vnodes_map[curvelink.getValueOfIdStart()];
      vedge.end_node_   = vnodes_map[curvelink.getValueOfIdEnd()];
      vedge.type_       = vrobot_route::v_link_type_t::CURVE;
      // Tính length thực của curve thay vì khoảng cách thẳng
      bezier::Bezier<3> bezierCurve(
        {bezier::Point(vnodes_map[curvelink.getValueOfIdStart()].pose_.x(),
                       vnodes_map[curvelink.getValueOfIdStart()].pose_.y()),
         bezier::Point(curvelink.getValueOfControlPoint1X(),
                       curvelink.getValueOfControlPoint1Y()),
         bezier::Point(curvelink.getValueOfControlPoint2X(),
                       curvelink.getValueOfControlPoint2Y()),
         bezier::Point(vnodes_map[curvelink.getValueOfIdEnd()].pose_.x(),
                       vnodes_map[curvelink.getValueOfIdEnd()].pose_.y())});

      // Tính arc length bằng cách sample curve
      double        arcLength = 0.0;
      const int     samples   = 100;
      bezier::Point prevPoint = bezierCurve.valueAt(0.0);

      for (int i = 1; i <= samples; ++i) {
        double        t            = static_cast<double>(i) / samples;
        bezier::Point currentPoint = bezierCurve.valueAt(t);
        double        dx           = currentPoint.x - prevPoint.x;
        double        dy           = currentPoint.y - prevPoint.y;
        arcLength += std::sqrt(dx * dx + dy * dy);
        prevPoint = currentPoint;
      }

      vedge.length_ = arcLength;

      vedge.control_points_.push_back(
        Eigen::Vector2d(curvelink.getValueOfControlPoint1X(),
                        curvelink.getValueOfControlPoint1Y()));
      vedge.control_points_.push_back(
        Eigen::Vector2d(curvelink.getValueOfControlPoint2X(),
                        curvelink.getValueOfControlPoint2Y()));

      vedge.max_vel_ =
        curvelink.getMaxVelocity() ? *curvelink.getMaxVelocity() : 2.0;
      vedge.width_ = 0.5;

      vedges_.push_back(vedge);
    }

    graph_ = std::make_shared<vrobot_route::GraphPose>(vnodes_, vedges_);

    auto graph_vis =
      graph_->visualizeGraph("map", this->now(), 0.1, 0.1, true, true);
    graph_publisher_->publish(graph_vis);

  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_logger(), "Error updating graph: %s", e.what());
    return false;
  }
  return true;
}

vrobot_route::GraphPose::PlanningResult VrobotRoute::getPath(
  Eigen::Vector2d start,
  int             end_id,
  std::string     map_name) {
  vrobot_route::GraphPose::PlanningConfig config;
  config.directThreshold = 0.3;
  config.maxLinkDistance = 0.5;
  config.enablePruning   = true;

  if (!update_graph(map_name)) {
    RCLCPP_ERROR(get_logger(), "Failed to update graph");
    return vrobot_route::GraphPose::PlanningResult();
  }

  vrobot_route::v_node_t end_node;
  for (const auto &vnode : vnodes_) {
    if (vnode.id_ == end_id) {
      end_node = vnode;
      break;
    }
  }

  return graph_->planPath(start, end_node, config);
}

nav_msgs::msg::Path VrobotRoute::toPath(
  const std::vector<vrobot_route::v_edge_t> &path_segments,
  const std::string                         &frame_id,
  const rclcpp::Time                        &timestamp,
  double                                     resolution) {
  auto vpath =
    graph_->pathSegmentsToVPath(path_segments, frame_id, timestamp, resolution);
  return graph_->toPath(vpath);
}

void VrobotRoute::publish_path(const nav_msgs::msg::Path &path) {
  path_publisher_->publish(path);
}

// Helper functions for angle calculations
double VrobotRoute::get_straight_segment_angle(
  const vrobot_route::v_edge_t &segment) {
  Eigen::Vector2d direction =
    segment.end_node_.pose_ - segment.start_node_.pose_;
  return std::atan2(direction.y(), direction.x());
}

double VrobotRoute::get_curve_start_angle(
  const vrobot_route::v_edge_t &curve_segment) {
  if (curve_segment.control_points_.size() < 2) {
    return get_straight_segment_angle(curve_segment);
  }

  // Hướng từ start node đến control point đầu tiên
  Eigen::Vector2d start_direction =
    curve_segment.control_points_[0] - curve_segment.start_node_.pose_;
  return std::atan2(start_direction.y(), start_direction.x());
}

double VrobotRoute::get_curve_end_angle(
  const vrobot_route::v_edge_t &curve_segment) {
  if (curve_segment.control_points_.size() < 2) {
    return get_straight_segment_angle(curve_segment);
  }

  // Hướng từ control point cuối đến end node
  Eigen::Vector2d end_direction =
    curve_segment.end_node_.pose_ - curve_segment.control_points_[1];
  return std::atan2(end_direction.y(), end_direction.x());
}

double VrobotRoute::calculate_angle_between_segments(
  const vrobot_route::v_edge_t &current_segment,
  const vrobot_route::v_edge_t &next_segment) {
  double current_end_angle, next_start_angle;

  // Tính góc cuối của segment hiện tại
  if (current_segment.type_ == vrobot_route::v_link_type_t::CURVE) {
    current_end_angle = get_curve_end_angle(current_segment);
  } else {
    current_end_angle = get_straight_segment_angle(current_segment);
  }

  // Tính góc đầu của segment tiếp theo
  if (next_segment.type_ == vrobot_route::v_link_type_t::CURVE) {
    next_start_angle = get_curve_start_angle(next_segment);
  } else {
    next_start_angle = get_straight_segment_angle(next_segment);
  }

  // Tính góc chênh lệch và normalize về [-π, π]
  double angle_diff = next_start_angle - current_end_angle;
  while (angle_diff > M_PI) {
    angle_diff -= 2 * M_PI;
  }
  while (angle_diff < -M_PI) {
    angle_diff += 2 * M_PI;
  }

  // QUAN TRỌNG: Tính góc rẽ thực tế thay vì góc chênh lệch
  // Góc rẽ = π - |angle_diff| để có:
  // - Góc tù (đi thẳng) → góc rẽ nhỏ
  // - Góc nhọn (rẽ gấp) → góc rẽ lớn
  double turning_angle = M_PI - std::abs(angle_diff);

  RCLCPP_DEBUG(
    get_logger(),
    "Angle diff: %.2f rad (%.1f deg), Turning angle: %.2f rad (%.1f deg)",
    angle_diff,
    angle_diff * 180.0 / M_PI,
    turning_angle,
    turning_angle * 180.0 / M_PI);

  return turning_angle;
}

std::vector<std::vector<vrobot_route::v_edge_t>> VrobotRoute::
  subdivide_sharp_turns(
    const std::vector<vrobot_route::v_edge_t> &path_segments,
    double                                     max_angle_threshold) {
  if (path_segments.size() <= 1) {
    return {path_segments};  // Return single sub-path
  }

  std::vector<std::vector<vrobot_route::v_edge_t>> subdivided_paths;
  std::vector<vrobot_route::v_edge_t>              current_subpath;

  for (size_t i = 0; i < path_segments.size(); ++i) {
    current_subpath.push_back(path_segments[i]);

    // Kiểm tra góc với segment tiếp theo (nếu có)
    if (i < path_segments.size() - 1) {
      double turning_angle =
        calculate_angle_between_segments(path_segments[i],
                                         path_segments[i + 1]);

      // Nếu góc rẽ quá gấp, tách thành sub-path riêng
      if (turning_angle < max_angle_threshold) {
        RCLCPP_INFO(get_logger(),
                    "Sharp turn detected: %.2f rad (%.1f deg) between segments "
                    "%zu and %zu",
                    turning_angle,
                    turning_angle * 180.0 / M_PI,
                    i,
                    i + 1);

        // Kết thúc sub-path hiện tại
        subdivided_paths.push_back(current_subpath);
        current_subpath.clear();
      }
    }
  }

  // Thêm sub-path cuối cùng nếu còn segments
  if (!current_subpath.empty()) {
    subdivided_paths.push_back(current_subpath);
  }

  RCLCPP_INFO(get_logger(),
              "Path subdivided from %zu segments into %zu sub-paths",
              path_segments.size(),
              subdivided_paths.size());

  return subdivided_paths;
}

}  // namespace vrobot_route

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // Node ros2
  auto node = std::make_shared<vrobot_route::VrobotRoute>();

  // spin
  rclcpp::spin(node);

  // shutdown
  rclcpp::shutdown();

  return 0;
}