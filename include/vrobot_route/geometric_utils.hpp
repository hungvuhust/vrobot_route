#pragma once

#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>

#include <vrobot_route/datastructor.hpp>

namespace vrobot_route {

class GeometricUtils : public virtual VGraph {
public:
  using Base = VGraph;

  // ========================================================================
  // NODE PROXIMITY OPERATIONS
  // ========================================================================

  /**
   * @brief Find nearest node to given pose
   * @param queryPose Query pose
   * @return NodeID of nearest node
   */
  v_node_t get_nearest_node(const Eigen::Vector2d &query_pose);

  /**
   * @brief Get distance to closest node
   * @param queryPose Query pose
   * @return Distance to nearest node
   */
  double get_distance_to_nearest_node(const Eigen::Vector2d &query_pose);

  // ========================================================================
  // LINK GEOMETRY OPERATIONS
  // ========================================================================

  /**
   * @brief Calculate distance from pose to line segment (link)
   * @param queryPose Query pose
   * @param nodeA First node of the link
   * @param nodeB Second node of the link
   * @return Distance from pose to line segment
   */
  double get_distance_to_line_segment(const Eigen::Vector2d &query_pose,
                                      const v_node_t        &nodeA,
                                      const v_node_t        &nodeB);

  /**
   * @brief Find projection point of pose onto line segment
   * @param queryPose Query pose
   * @param nodeA First node of the link
   * @param nodeB Second node of the link
   * @return Projection point on line segment
   */
  Eigen::Vector2d
  get_projection_point_on_line_segment(const Eigen::Vector2d &query_pose,
                                       const v_node_t        &nodeA,
                                       const v_node_t        &nodeB);

  /**
   * @brief Find closest links to given pose with distance filtering
   * @param queryPose Query pose
   * @param maxLinks Maximum number of links to return
   * @param maxDistance Maximum distance threshold for links
   * @return Vector of (nodeA, nodeB, distance) sorted by distance
   */
  std::vector<std::tuple<v_node_t, v_node_t, double>>
  get_closest_links(const Eigen::Vector2d &query_pose, int max_links,
                    double max_distance);

  // ========================================================================
  // INTERNAL GEOMETRIC CALCULATIONS
  // ========================================================================

  /**
   * @brief Calculate distance from point to line segment
   * @param point Query point
   * @param lineStart Start point of line segment
   * @param lineEnd End point of line segment
   * @return Distance from point to line segment
   */
  double get_distance_to_line_segment(const Eigen::Vector2d &point,
                                      const Eigen::Vector2d &line_start,
                                      const Eigen::Vector2d &line_end);

  /**
   * @brief Calculate projection of point onto line segment
   * @param point Query point
   * @param lineStart Start point of line segment
   * @param lineEnd End point of line segment
   * @return Projection point on line segment
   */
  Eigen::Vector2d
  get_projection_point_on_line_segment(const Eigen::Vector2d &point,
                                       const Eigen::Vector2d &line_start,
                                       const Eigen::Vector2d &line_end);
};
} // namespace vrobot_route