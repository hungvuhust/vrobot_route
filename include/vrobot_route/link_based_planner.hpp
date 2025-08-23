#pragma once

#include <vrobot_route/pathfinding.hpp>
#include <vrobot_route/scoring.hpp>

namespace vrobot_route {

class LinkBasedPlanner : public virtual Pathfinding,
                         public virtual ScoringSystem {
public:
  using Base        = Pathfinding;
  using ScoringBase = ScoringSystem;
  using PathSegment = v_edge_t;

  // ========================================================================
  // CORE LINK-BASED ALGORITHMS
  // ========================================================================
  /**
   * @brief Advanced path planning with modular link-based approach
   * @param startPose Starting pose
   * @param targetNode Target node ID
   * @param directThreshold Distance threshold for direct path
   * @param maxLinks Maximum number of links to evaluate
   * @param linkDistanceWeight Weight for link distance in scoring
   * @param maxLinkDistance Maximum distance to consider links
   * @param graphDistanceWeight Weight for graph distance in scoring
   * @return Pair of (path_segments, total_distance)
   */
  std::pair<std::vector<PathSegment>, std::optional<double>>
  dijkstra_with_modular_link_approach(
    const Eigen::Vector2d &startPose,
    const v_node_t        &targetNode,
    double                 directThreshold    = 1.5,
    size_t                 maxLinks           = 3,
    double                 linkDistanceWeight = 2.0,
    double maxLinkDistance     = std::numeric_limits<double>::infinity(),
    double graphDistanceWeight = 1.0) const;

  /**
   * @brief Greedy strategy: find closest links within tolerance, evaluate complete paths, choose shortest
   * @param startPose Starting pose
   * @param targetNode Target node ID
   * @param tolerance Maximum distance to consider links
   * @param maxLinks Maximum number of links to evaluate (default 4)
   * @return Pair of (path_segments, total_distance)
   */
  std::pair<std::vector<PathSegment>, std::optional<double>>
  dijkstra_with_greedy_strategy(
    const Eigen::Vector2d &startPose,
    const v_node_t        &targetNode,
    double                 tolerance = 0.5,
    size_t                 maxLinks = 4) const;

  // ========================================================================
  // ENHANCED LINK ACCESS
  // ========================================================================
  /**
   * @brief Enhanced path planning with link access for distant poses
   * @param startPose Starting pose
   * @param targetNode Target node ID
   * @param distanceThreshold Threshold for using link-based approach
   * @param maxLinks Maximum number of links to consider
   * @param linkDistanceWeight Weight for link distance scoring
   * @return Pair of (path_segments, total_distance)
   */
  std::pair<std::vector<PathSegment>, std::optional<double>>
  dijkstra_with_link_access(const Eigen::Vector2d &startPose,
                            const v_node_t        &targetNode,
                            double                 distanceThreshold = 0.5,
                            size_t                 maxLinks          = 3,
                            double linkDistanceWeight = 2.0) const;

  // ========================================================================
  // SIMPLE LINK ACCESS
  // ========================================================================
  /**
   * @brief Simple link access approach: start → closest point on link →
   * destination
   * @param startPose Starting pose
   * @param targetNode Target node ID
   * @param maxLinks Maximum number of links to consider
   * @return Pair of (path_segments, total_distance)
   */
  std::pair<std::vector<PathSegment>, std::optional<double>>
  dijkstra_with_simple_link_access(const Eigen::Vector2d &startPose,
                                   const v_node_t        &targetNode,
                                   size_t                 maxLinks = 3) const;

  // ========================================================================
  // LINK FOLLOWING APPROACHES
  // ========================================================================
  /**
   * @brief Link following approach where robot follows link topology
   * @param startPose Starting pose
   * @param targetNode Target node ID
   * @param maxLinks Maximum number of links to consider
   * @return Pair of (path_segments, total_distance)
   */
  std::pair<std::vector<PathSegment>, std::optional<double>>
  dijkstra_with_link_following(const Eigen::Vector2d &startPose,
                               const v_node_t        &targetNode,
                               size_t                 maxLinks = 3) const;

  /**
   * @brief Smart link following with adaptive path selection
   * @param startPose Starting pose
   * @param targetNode Target node ID
   * @param maxLinks Maximum number of links to consider
   * @param adaptiveWeight Adaptive weight factor
   * @return Pair of (path_segments, total_distance)
   */
  std::pair<std::vector<PathSegment>, std::optional<double>>
  dijkstra_with_smart_link_following(const Eigen::Vector2d &startPose,
                                     const v_node_t        &targetNode,
                                     size_t                 maxLinks = 3,
                                     double adaptiveWeight = 1.5) const;

private:
  // ========================================================================
  // HELPER METHODS
  // ========================================================================
  /**
   * @brief Helper method to build path through link
   * @param startPose Starting pose
   * @param linkStart Start node of link
   * @param linkEnd End node of link
   * @param targetNode Target node
   * @return Path segments and distance
   */
  std::pair<std::vector<PathSegment>, std::optional<double>>
  build_path_through_link(const Eigen::Vector2d &startPose,
                          const v_node_t        &linkStart,
                          const v_node_t        &linkEnd,
                          const v_node_t        &targetNode) const;
};

}  // namespace vrobot_route