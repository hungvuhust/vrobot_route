#pragma once

#include <vrobot_route/graph_pose.hpp>

#include <rclcpp/rclcpp.hpp>

namespace vrobot_route {

class VrobotRoute : public rclcpp::Node {
public:
  VrobotRoute();
  ~VrobotRoute();

  void init_db();
  void init_publishers();

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
};
}  // namespace vrobot_route
