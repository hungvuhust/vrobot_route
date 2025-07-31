#pragma once

#include <vrobot_route/datastructor.hpp>
#include <vrobot_route/geometric_utils.hpp>

namespace vrobot_route {

class Pathfinding : public virtual GeometricUtils {
public:
  using Base        = GeometricUtils;
  using PathSegment = v_edge_t;

  // ========================================================================
  // DIJKSTRA ALGORITHM
  // ========================================================================

  /**
   * @brief Find shortest path between two nodes using Dijkstra algorithm
   * @param startNode Starting node ID
   * @param targetNode Target node ID
   * @return Pair of (path_segments, total_distance)
   */
  std::pair<std::vector<PathSegment>, std::optional<double>>
  dijkstra(const v_node_t &start_node, const v_node_t &target_node);

  /**
   * @brief Find path from arbitrary pose to target node
   * @param startPose Starting pose (not necessarily at a node)
   * @param targetNode Target node ID
   * @param maxDistanceToGraph Maximum allowed distance from pose to closest
   * node
   * @return Pair of (path_segments, total_distance)
   */
  std::pair<std::vector<PathSegment>, std::optional<double>>
  dijkstra_from_pose(const Eigen::Vector2d &start_pose,
                     const v_node_t &target_node, double max_distance_to_graph);

  /**
   * @brief Find path between two arbitrary poses
   * @param startPose Starting pose
   * @param targetPose Target pose
   * @param maxDistanceToGraph Maximum allowed distance from poses to closest
   * nodes
   * @return Pair of (path_segments, total_distance)
   */
  std::pair<std::vector<PathSegment>, std::optional<double>>
  dijkstra_pose_to_pose(
      const Eigen::Vector2d &start_pose, const Eigen::Vector2d &target_pose,
      double maxDistanceToGraph = std::numeric_limits<double>::infinity());

protected:
  // ========================================================================
  // HELPER FUNCTIONS
  // ========================================================================

  /**
   * @brief Calculate total distance of path segments
   * @param pathSegments Vector of path segments
   * @return Total distance
   */
  double calculate_path_distance(const std::vector<PathSegment> &path_segments);

  /**
   * @brief Check if path is valid (all segments connected)
   * @param pathSegments Vector of path segments
   * @return True if path is valid
   */
  bool is_valid_path(const std::vector<PathSegment> &path_segments);
};

} // namespace vrobot_route