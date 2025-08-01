#pragma once

#include <rclcpp/rclcpp.hpp>
#include <vrobot_local_planner/msg/path.hpp>
#include <vrobot_local_planner/msg/planner_pose.hpp>
#include <vrobot_route/datastructor.hpp>
#include <vrobot_route/geometric_utils.hpp>
#include <vrobot_route/link_based_planner.hpp>
#include <vrobot_route/nav_conversion.hpp>
#include <vrobot_route/pathfinding.hpp>

namespace vrobot_route {

class GraphPose : public virtual vrobot_route::VGraph,
                  public virtual vrobot_route::GeometricUtils,
                  public virtual vrobot_route::Pathfinding,
                  public virtual vrobot_route::LinkBasedPlanner,
                  public virtual vrobot_route::NavConversion {
public:
  using Base        = vrobot_route::VGraph;
  using PathSegment = vrobot_route::v_edge_t;
  // ========================================================================
  // CONSTRUCTORS
  // ========================================================================

  /**
   * @brief Default constructor
   */
  GraphPose() = default;

  /**
   * @brief Constructor with nodes and edges
   * @param vnodes Vector of nodes
   * @param vedges Vector of edges
   */
  GraphPose(const std::vector<vrobot_route::v_node_t> &vnodes,
            const std::vector<vrobot_route::v_edge_t> &vedges)
      : Base(vnodes, vedges) {}

  // ========================================================================
  // HIGH-LEVEL PATH PLANNING INTERFACE
  // ========================================================================

  /**
   * @brief Comprehensive path planning with automatic algorithm selection
   * @param startPose Starting pose
   * @param targetNode Target node ID
   * @param config Planning configuration
   * @return Planning result with path and metadata
   */
  struct PlanningConfig {
    double directThreshold = 1.5; // distance threshold for direct path
    double distanceThreshold =
        0.5;                       // distance threshold for link-based approach
    size_t maxLinks           = 4; // maximum number of links to consider
    double linkDistanceWeight = 2.0; // weight for link distance
    double maxLinkDistance =
        std::numeric_limits<double>::infinity(); // maximum distance for
                                                 // link-based approach
    double graphDistanceWeight = 1.0;            // weight for graph distance
    bool   enableLinkBased     = true;           // enable link-based approach
    bool enablePruning = false; // Disabled by default for link-based approaches
    double interpolationResolution = 0.01; // interpolation resolution for path
  };

  struct PlanningResult {
    std::vector<PathSegment>      pathSegments;
    std::optional<double>         totalDistance;
    std::string                   algorithmUsed;
    std::string                   errorMessage;
    std::map<std::string, double> metadata;
    bool                          success = false;
  };

  PlanningResult planPath(const Eigen::Vector2d &startPose,
                          const v_node_t        &targetNode,
                          const PlanningConfig  &config) const;

  /**
   * @brief Convert planning result to ROS2 nav_msgs::msg::Path
   * @param result Planning result
   * @param frameId Frame ID for the path
   * @param timestamp Timestamp for the path
   * @param resolution Interpolation resolution (meters)
   * @return ROS2 navigation path
   */
  vrobot_local_planner::msg::Path planningResultToVPath(
      const PlanningResult &result, const std::string &frameId,
      const rclcpp::Time &timestamp, double resolution = 0.01) const;

  /**
   * @brief Convert path segments to velocity path segments using edge
   * velocities
   * @param pathSegments Vector of path segments
   * @return Vector of velocity path segments
   */
  vrobot_local_planner::msg::Path
  pathSegmentsToVPath(const std::vector<PathSegment> &pathSegments,
                      const std::string &frameId, const rclcpp::Time &timestamp,
                      double resolution = 0.01) const;
};

} // namespace vrobot_route