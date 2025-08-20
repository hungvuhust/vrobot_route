#include "vrobot_route/datastructor.hpp"
#include <vrobot_route/graph_pose.hpp>

namespace vrobot_route {

GraphPose::PlanningResult GraphPose::planPath(
  const Eigen::Vector2d &startPose,
  const v_node_t        &targetNode,
  const PlanningConfig  &config) const {
  std::cout << "Plan path from (" << startPose.transpose() << ") to "
            << targetNode.id_ << std::endl;
  PlanningResult result;

  if (!Base::has_node(targetNode)) {
    result.algorithmUsed = "INVALID_TARGET";
    result.errorMessage  = "Target node " + std::to_string(targetNode.id_) +
                          " does not exist in graph";
    return result;
  }

  const Eigen::Vector2d &targetPose     = Base::get_pose(targetNode);
  double                 directDistance = (startPose - targetPose).norm();

  // Check for direct path first
  if (directDistance <= config.directThreshold) {
    v_edge_t vedge = create_virtual_edge(startPose, targetPose);

    result.pathSegments = {vedge};

    result.totalDistance               = directDistance;
    result.algorithmUsed               = "DIRECT";
    result.success                     = true;
    result.metadata["direct_distance"] = directDistance;
    return result;
  }

  // Check if close enough for standard Dijkstra
  double distanceToClosest = this->get_distance_to_nearest_node(startPose);
  if (!config.enableLinkBased ||
      distanceToClosest <= config.distanceThreshold) {
    auto [path, distance] =
      this->dijkstra_from_pose(startPose, targetNode, config.maxLinkDistance);
    if (distance) {
      result.pathSegments                    = path;
      result.totalDistance                   = distance;
      result.algorithmUsed                   = "DIJKSTRA_FROM_POSE";
      result.success                         = true;
      result.metadata["distance_to_closest"] = distanceToClosest;
      return result;
    }
  }

  // Use advanced link-based approach
  auto [linkPath, linkDistance] =
    this->dijkstra_with_modular_link_approach(startPose,
                                              targetNode,
                                              config.directThreshold,
                                              config.maxLinks,
                                              config.linkDistanceWeight,
                                              config.maxLinkDistance,
                                              config.graphDistanceWeight);

  if (linkDistance) {
    result.pathSegments                  = linkPath;
    result.totalDistance                 = linkDistance;
    result.algorithmUsed                 = "MODULAR_LINK_BASED";
    result.success                       = true;
    result.metadata["link_weight"]       = config.linkDistanceWeight;
    result.metadata["max_link_distance"] = config.maxLinkDistance;
    return result;
  }

  // Fallback to basic approach
  auto [fallbackPath, fallbackDistance] =
    this->dijkstra_from_pose(startPose, targetNode, config.maxLinkDistance);
  if (fallbackDistance) {
    result.pathSegments  = fallbackPath;
    result.totalDistance = fallbackDistance;
    result.algorithmUsed = "DIJKSTRA_FALLBACK";
    result.success       = true;
    return result;
  }

  result.algorithmUsed = "FAILED";
  result.errorMessage =
    "All pathfinding algorithms failed. Distance to closest node: " +
    std::to_string(distanceToClosest) +
    ", Max distance threshold: " + std::to_string(config.maxLinkDistance) +
    ", Direct distance: " + std::to_string(directDistance) +
    ", Direct threshold: " + std::to_string(config.directThreshold);
  return result;
}

vrobot_local_planner::msg::Path GraphPose::planningResultToVPath(
  const PlanningResult &result,
  const std::string    &frameId,
  const rclcpp::Time   &timestamp,
  double                resolution) const {
  if (!result.success) {
    vrobot_local_planner::msg::Path emptyPath;
    emptyPath.header.frame_id = frameId;
    emptyPath.header.stamp    = timestamp;
    return emptyPath;
  }

  return this->toVPath(result.pathSegments, frameId, timestamp, resolution);
}

vrobot_local_planner::msg::Path GraphPose::pathSegmentsToVPath(
  const std::vector<PathSegment> &pathSegments,
  const std::string              &frameId,
  const rclcpp::Time             &timestamp,
  double                          resolution) const {
  return this->toVPath(pathSegments, frameId, timestamp, resolution);
}

}  // namespace vrobot_route