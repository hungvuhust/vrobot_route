#include <vrobot_route/geometric_utils.hpp>
#include <cmath>
#include <algorithm>

namespace vrobot_route {

v_node_t GeometricUtils::get_nearest_node(
  const Eigen::Vector2d &query_pose) const {
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

double GeometricUtils::get_distance_to_line_segment(
  const Eigen::Vector2d &query_pose,
  const v_node_t        &nodeA,
  const v_node_t        &nodeB) const {
  return (query_pose -
          get_projection_point_on_line_segment(query_pose, nodeA, nodeB))
    .norm();
}

Eigen::Vector2d GeometricUtils::get_projection_point_on_line_segment(
  const Eigen::Vector2d &query_pose,
  const v_node_t        &nodeA,
  const v_node_t        &nodeB) const {
  Eigen::Vector2d line_dir = (nodeB.pose_ - nodeA.pose_).normalized();
  double          t        = (query_pose - nodeA.pose_).dot(line_dir);
  return nodeA.pose_ + t * line_dir;
}

double GeometricUtils::get_distance_to_curve(const Eigen::Vector2d &query_pose,
                                             const v_edge_t &edge) const {
  // Kiểm tra nếu không phải đường cong hoặc không có đủ control points
  if (edge.type_ != v_link_type_t::CURVE || edge.control_points_.size() < 2) {
    // Fallback to line segment calculation
    return get_distance_to_line_segment(query_pose,
                                        edge.start_node_,
                                        edge.end_node_);
  }

  // Tạo Bezier curve từ control points (cubic Bezier)
  bezier::Bezier<3> curve(
    {bezier::Point(edge.start_node_.pose_.x(), edge.start_node_.pose_.y()),
     bezier::Point(edge.control_points_[0].x(), edge.control_points_[0].y()),
     bezier::Point(edge.control_points_[1].x(), edge.control_points_[1].y()),
     bezier::Point(edge.end_node_.pose_.x(), edge.end_node_.pose_.y())});

  // Tìm điểm gần nhất trên curve bằng sampling
  double    min_distance = std::numeric_limits<double>::max();
  const int samples      = 100;  // Độ chính xác có thể điều chỉnh

  for (int i = 0; i <= samples; ++i) {
    double          t     = static_cast<double>(i) / samples;
    auto            point = curve.valueAt(t);
    Eigen::Vector2d curve_point(point.x, point.y);
    double          dist = (query_pose - curve_point).norm();
    min_distance         = std::min(min_distance, dist);
  }

  return min_distance;
}

std::pair<Eigen::Vector2d, double> GeometricUtils::
  get_projection_point_on_curve(const Eigen::Vector2d &query_pose,
                                const v_edge_t        &edge) const {
  // Kiểm tra nếu không phải đường cong hoặc không có đủ control points
  if (edge.type_ != v_link_type_t::CURVE || edge.control_points_.size() < 2) {
    // Fallback to line segment calculation
    Eigen::Vector2d proj =
      get_projection_point_on_line_segment(query_pose,
                                           edge.start_node_,
                                           edge.end_node_);
    return {proj, 0.5};  // Return middle parameter as default
  }

  // Tạo Bezier curve từ control points
  bezier::Bezier<3> curve(
    {bezier::Point(edge.start_node_.pose_.x(), edge.start_node_.pose_.y()),
     bezier::Point(edge.control_points_[0].x(), edge.control_points_[0].y()),
     bezier::Point(edge.control_points_[1].x(), edge.control_points_[1].y()),
     bezier::Point(edge.end_node_.pose_.x(), edge.end_node_.pose_.y())});

  // Tìm điểm gần nhất và parameter t tương ứng
  double          min_distance = std::numeric_limits<double>::max();
  double          best_t       = 0.0;
  Eigen::Vector2d best_point;
  const int       samples = 100;

  for (int i = 0; i <= samples; ++i) {
    double          t     = static_cast<double>(i) / samples;
    auto            point = curve.valueAt(t);
    Eigen::Vector2d curve_point(point.x, point.y);
    double          dist = (query_pose - curve_point).norm();

    if (dist < min_distance) {
      min_distance = dist;
      best_t       = t;
      best_point   = curve_point;
    }
  }

  return {best_point, best_t};
}

const v_edge_t *GeometricUtils::find_edge_between_nodes(
  const v_node_t &nodeA,
  const v_node_t &nodeB) const {
  // Tìm trong adjacency list của nodeA
  if (adj_list_.find(nodeA) != adj_list_.end()) {
    for (const auto &[neighbor, edge] : adj_list_.at(nodeA)) {
      if (neighbor == nodeB) {
        return &edge;
      }
    }
  }

  // Tìm trong adjacency list của nodeB (để xử lý directed graph)
  if (adj_list_.find(nodeB) != adj_list_.end()) {
    for (const auto &[neighbor, edge] : adj_list_.at(nodeB)) {
      if (neighbor == nodeA) {
        return &edge;
      }
    }
  }

  return nullptr;
}

Eigen::Vector2d GeometricUtils::get_smart_projection_point(
  const Eigen::Vector2d &query_pose,
  const v_node_t        &nodeA,
  const v_node_t        &nodeB) const {
  // Tìm edge giữa hai nodes
  const v_edge_t *edge = find_edge_between_nodes(nodeA, nodeB);

  if (edge && edge->type_ == v_link_type_t::CURVE) {
    // Sử dụng projection cho đường cong
    auto [proj_point, t] = get_projection_point_on_curve(query_pose, *edge);
    return proj_point;
  } else {
    // Sử dụng projection cho đường thẳng
    return get_projection_point_on_line_segment(query_pose, nodeA, nodeB);
  }
}

std::vector<std::tuple<v_node_t, v_node_t, double>> GeometricUtils::
  get_closest_links(const Eigen::Vector2d &query_pose,
                    size_t                 max_links,
                    double                 max_distance) const {
  std::vector<std::tuple<v_node_t, v_node_t, double>> closest_links;
  for (const auto &node : nodes_) {
    for (const auto &pair : adj_list_.at(node)) {
      if (pair.first == node || pair.second.start_node_ == node) {
        double dist;

        // Kiểm tra loại đường và tính khoảng cách tương ứng
        if (pair.second.type_ == v_link_type_t::CURVE) {
          dist = get_distance_to_curve(query_pose, pair.second);
        } else {
          // STRAIGHT hoặc default
          dist = get_distance_to_line_segment(query_pose,
                                              pair.first,
                                              pair.second.start_node_);
        }

        if (dist <= max_distance) {
          closest_links.push_back(
            std::make_tuple(pair.first, pair.second.start_node_, dist));
        }
      }
    }
  }
  std::sort(closest_links.begin(),
            closest_links.end(),
            [](const auto &a, const auto &b) {
              return std::get<2>(a) < std::get<2>(b);
            });

  if (closest_links.size() > max_links) {
    closest_links.resize(max_links);
  }

  return closest_links;
}

double GeometricUtils::get_distance_to_line_segment(
  const Eigen::Vector2d &point,
  const Eigen::Vector2d &line_start,
  const Eigen::Vector2d &line_end) const {
  Eigen::Vector2d line_dir = (line_end - line_start).normalized();
  double          t        = (point - line_start).dot(line_dir);
  return (point - (line_start + t * line_dir)).norm();
}

Eigen::Vector2d GeometricUtils::get_projection_point_on_line_segment(
  const Eigen::Vector2d &point,
  const Eigen::Vector2d &line_start,
  const Eigen::Vector2d &line_end) const {
  Eigen::Vector2d line_dir = (line_end - line_start).normalized();
  double          t        = (point - line_start).dot(line_dir);
  return line_start + t * line_dir;
}

}  // namespace vrobot_route