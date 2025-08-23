#pragma once

#include <vrobot_route/graph_pose.hpp>

#include <rclcpp/rclcpp.hpp>
#include <vrobot_route/action/move_to_pose.hpp>
#include <vrobot_route/simple_action_server.hpp>
#include "vrobot_local_planner/action/v_follow_path.hpp"
#include <condition_variable>
#include <mutex>
#include <thread>

namespace vrobot_route {

class VrobotRoute : public rclcpp::Node {
public:
  VrobotRoute();
  ~VrobotRoute();

  void init_db();
  void init_publishers();
  void init_action_server();

  bool update_graph(std::string map_name);

  vrobot_route::GraphPose::PlanningResult getPath(Eigen::Vector2d start,
                                                  int             end_id,
                                                  std::string     map_name);

  nav_msgs::msg::Path toPath(
    const std::vector<vrobot_route::v_edge_t> &path_segments,
    const std::string                         &frame_id,
    const rclcpp::Time                        &timestamp,
    double                                     resolution);

  void publish_path(const nav_msgs::msg::Path &path);

  // Sharp turn subdivision methods
  std::vector<std::vector<vrobot_route::v_edge_t>> subdivide_sharp_turns(
    const std::vector<vrobot_route::v_edge_t> &path_segments,
    double max_angle_threshold = M_PI / 180 * 100);  // 100 degrees default

protected:
  using Action       = vrobot_route::action::MoveToPose;
  using ActionServer = vrobot_route::SimpleActionServer<Action>;
  std::unique_ptr<ActionServer> action_server_;

  void execute_move_to_pose();
  void send_follow_path_goal(const std::vector<vrobot_route::v_edge_t> &path,
                             bool is_last_goal = false);

  rclcpp_action::Client<vrobot_local_planner::action::VFollowPath>::SharedPtr
                          v_follow_path_client_;
  std::condition_variable cv_follow_path_goal_;
  std::mutex              mtx_follow_path_goal_;
  bool                    is_goal_completed_{false};

private:
  // Helper methods for angle calculations
  double get_straight_segment_angle(const vrobot_route::v_edge_t &segment);
  double get_curve_start_angle(const vrobot_route::v_edge_t &curve_segment);
  double get_curve_end_angle(const vrobot_route::v_edge_t &curve_segment);
  double calculate_angle_between_segments(
    const vrobot_route::v_edge_t &current_segment,
    const vrobot_route::v_edge_t &next_segment);
  std::shared_ptr<vrobot_route::GraphPose> graph_;
  std::vector<vrobot_route::v_node_t>      vnodes_;
  std::vector<vrobot_route::v_edge_t>      vedges_;

  vrobot_route::DbManager::Ptr db_manager_;
  std::string                  connection_info_{
    "host=127.0.0.1 port=5432 "
                     "dbname=amr_01 user=amr "
                     "password=1234512345"};

  std::string map_name_{"mapmoi"};

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
                                                    graph_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<vrobot_local_planner::msg::Path>::SharedPtr
    vpath_publisher_;
};
}  // namespace vrobot_route
