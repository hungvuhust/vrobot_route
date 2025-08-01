#include <vrobot_route/link_based_planner.hpp>

namespace vrobot_route {

std::pair<std::vector<v_edge_t>, std::optional<double>>
LinkBasedPlanner::dijkstra_with_modular_link_approach(
    const Eigen::Vector2d &startPose, const v_node_t &targetNode,
    double directThreshold, size_t maxLinks, double linkDistanceWeight,
    double maxLinkDistance, double graphDistanceWeight) const {

  if (!Base::has_node(targetNode)) {
    return {{}, std::nullopt};
  }

  const Eigen::Vector2d &targetPose = Base::get_pose(targetNode);

  // Check for direct path first
  double directDistance = (startPose - targetPose).norm();
  if (directDistance <= directThreshold) {
    std::vector<v_edge_t> directPath = {
        create_virtual_edge(startPose, targetPose)};
    return {directPath, directDistance};
  }

  // Step 1: Find closest links with distance filtering
  auto closestLinks =
      Base::get_closest_links(startPose, maxLinks, maxLinkDistance);
  if (closestLinks.empty()) {
    return Base::dijkstra_from_pose(startPose, targetNode, maxLinkDistance);
  }

  // Step 2: Evaluate each link using weighted scoring
  std::optional<std::vector<PathSegment>> bestPath;
  std::optional<double>                   bestScore;

  for (const auto &[linkStart, linkEnd, linkDistance] : closestLinks) {
    // Calculate path from link to destination
    auto [linkToTarget, linkToTargetDist] = Base::dijkstra(linkEnd, targetNode);
    if (!linkToTargetDist) {
      continue;
    }

    // Calculate weighted score
    double score =
        this->calculate_weighted_score(linkDistance, *linkToTargetDist,
                                       linkDistanceWeight, graphDistanceWeight);

    if (!bestScore || score < *bestScore) {
      // Step 3: Build complete path
      Eigen::Vector2d projectionPoint =
          Base::get_projection_point_on_line_segment(startPose, linkStart,
                                                     linkEnd);
      const Eigen::Vector2d &linkEndPose = Base::get_pose(linkEnd);

      std::vector<PathSegment> candidatePath;
      candidatePath.emplace_back(
          create_virtual_edge(startPose, projectionPoint));
      candidatePath.emplace_back(
          create_virtual_edge(projectionPoint, linkEndPose));
      candidatePath.insert(candidatePath.end(), linkToTarget.begin(),
                           linkToTarget.end());

      bestPath  = candidatePath;
      bestScore = score;
    }
  }

  if (!bestPath) {
    return Base::dijkstra_from_pose(startPose, targetNode, maxLinkDistance);
  }

  // Calculate actual total distance
  double totalDistance = Base::calculate_path_distance(*bestPath);
  return {*bestPath, totalDistance};
}

} // namespace vrobot_route