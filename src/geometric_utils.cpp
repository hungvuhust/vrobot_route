#include <vrobot_route/geometric_utils.hpp>

namespace vrobot_route {

v_node_t
GeometricUtils::get_nearest_node(const Eigen::Vector2d &query_pose) const {
  double   min_dist = std::numeric_limits<double>::max();
  v_node_t nearest_node;
  for (const auto &node : nodes_) {
    double dist = (node.pose_ - query_pose).norm();
    if (dist < min_dist) {
      min_dist     = dist;
      nearest_node = node;
    }
  }
  return nearest_node;
}

double GeometricUtils::get_distance_to_nearest_node(
    const Eigen::Vector2d &query_pose) const {
  v_node_t nearest_node = get_nearest_node(query_pose);
  return (query_pose - get_pose(nearest_node)).norm();
}

double
GeometricUtils::get_distance_to_line_segment(const Eigen::Vector2d &query_pose,
                                             const v_node_t        &nodeA,
                                             const v_node_t &nodeB) const {
  return (query_pose -
          get_projection_point_on_line_segment(query_pose, nodeA, nodeB))
      .norm();
}

Eigen::Vector2d GeometricUtils::get_projection_point_on_line_segment(
    const Eigen::Vector2d &query_pose, const v_node_t &nodeA,
    const v_node_t &nodeB) const {
  Eigen::Vector2d line_dir = (nodeB.pose_ - nodeA.pose_).normalized();
  double          t        = (query_pose - nodeA.pose_).dot(line_dir);
  return nodeA.pose_ + t * line_dir;
}

std::vector<std::tuple<v_node_t, v_node_t, double>>
GeometricUtils::get_closest_links(const Eigen::Vector2d &query_pose,
                                  int max_links, double max_distance) const {
  std::vector<std::tuple<v_node_t, v_node_t, double>> closest_links;
  for (const auto &node : nodes_) {
    for (const auto &pair : adj_list_.at(node)) {
      if (pair.first == node || pair.second.start_node_ == node) {
        double dist = get_distance_to_line_segment(query_pose, pair.first,
                                                   pair.second.start_node_);
        if (dist <= max_distance) {
          closest_links.push_back(
              std::make_tuple(pair.first, pair.second.start_node_, dist));
        }
      }
    }
  }
  std::sort(closest_links.begin(), closest_links.end(),
            [](const auto &a, const auto &b) {
              return std::get<2>(a) < std::get<2>(b);
            });
  return closest_links;
}

double GeometricUtils::get_distance_to_line_segment(
    const Eigen::Vector2d &point, const Eigen::Vector2d &line_start,
    const Eigen::Vector2d &line_end) const {
  Eigen::Vector2d line_dir = (line_end - line_start).normalized();
  double          t        = (point - line_start).dot(line_dir);
  return (point - (line_start + t * line_dir)).norm();
}

Eigen::Vector2d GeometricUtils::get_projection_point_on_line_segment(
    const Eigen::Vector2d &point, const Eigen::Vector2d &line_start,
    const Eigen::Vector2d &line_end) const {
  Eigen::Vector2d line_dir = (line_end - line_start).normalized();
  double          t        = (point - line_start).dot(line_dir);
  return line_start + t * line_dir;
}

} // namespace vrobot_route