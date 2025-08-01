#include "vrobot_route/datastructor.hpp"
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

std::pair<std::vector<v_edge_t>, std::optional<double>>
LinkBasedPlanner::dijkstra_with_link_access(const Eigen::Vector2d &startPose,
                                            const v_node_t        &targetNode,
                                            double distanceThreshold,
                                            size_t maxLinks,
                                            double linkDistanceWeight) const {
  if (!Base::has_node(targetNode)) {
    return {{}, std::nullopt};
  }

  // Check if close enough to use standard approach
  double distanceToClosest = Base::get_distance_to_nearest_node(startPose);
  if (distanceToClosest <= distanceThreshold) {
    return Base::dijkstra_from_pose(startPose, targetNode);
  }

  // Use link-based approach for distant poses
  return dijkstra_with_modular_link_approach(
      startPose, targetNode, distanceThreshold, maxLinks, linkDistanceWeight);
}

std::pair<std::vector<v_edge_t>, std::optional<double>>
LinkBasedPlanner::dijkstra_with_simple_link_access(
    const Eigen::Vector2d &startPose, const v_node_t &targetNode,
    size_t maxLinks) const {

  if (!Base::has_node(targetNode)) {
    return {{}, std::nullopt};
  }

  // Find closest links
  auto closestLinks = Base::get_closest_links(startPose, maxLinks);
  if (closestLinks.empty()) {
    return Base::dijkstra_from_pose(startPose, targetNode);
  }

  std::optional<std::vector<PathSegment>> bestPath;
  std::optional<double>                   bestDistance;

  for (const auto &[linkStart, linkEnd, linkDistance] : closestLinks) {
    // Project onto link
    Eigen::Vector2d projectionPoint =
        Base::get_projection_point_on_line_segment(startPose, linkStart,
                                                   linkEnd);

    // Find closer node on the link
    const Eigen::Vector2d &startNodePose = Base::get_pose(linkStart);
    const Eigen::Vector2d &endNodePose   = Base::get_pose(linkEnd);

    v_node_t accessNode = ((projectionPoint - startNodePose).norm() <
                           (projectionPoint - endNodePose).norm())
                              ? linkStart
                              : linkEnd;

    // Path from access node to target
    auto [nodeToTarget, nodeToTargetDist] =
        Base::dijkstra(accessNode, targetNode);
    if (!nodeToTargetDist) {
      continue;
    }

    // Build complete path
    std::vector<PathSegment> candidatePath;
    candidatePath.emplace_back(create_virtual_edge(startPose, projectionPoint));
    candidatePath.emplace_back(
        create_virtual_edge(projectionPoint, Base::get_pose(accessNode)));

    candidatePath.insert(candidatePath.end(), nodeToTarget.begin(),
                         nodeToTarget.end());

    double totalDistance = Base::calculate_path_distance(candidatePath);

    if (!bestDistance || totalDistance < *bestDistance) {
      bestPath     = candidatePath;
      bestDistance = totalDistance;
    }
  }

  if (!bestPath) {
    return Base::dijkstra_from_pose(startPose, targetNode);
  }

  return {*bestPath, *bestDistance};
}

std::pair<std::vector<v_edge_t>, std::optional<double>>
LinkBasedPlanner::dijkstra_with_link_following(const Eigen::Vector2d &startPose,
                                               const v_node_t &targetNode,
                                               size_t          maxLinks) const {
  if (!Base::has_node(targetNode)) {
    return {{}, std::nullopt};
  }

  auto closestLinks = Base::get_closest_links(startPose, maxLinks);
  if (closestLinks.empty()) {
    return Base::dijkstra_from_pose(startPose, targetNode);
  }

  std::optional<std::vector<PathSegment>> bestPath;
  std::optional<double>                   bestDistance;

  for (const auto &[linkStart, linkEnd, linkDistance] : closestLinks) {
    // Try both directions on the link
    auto result1 =
        build_path_through_link(startPose, linkStart, linkEnd, targetNode);
    auto result2 =
        build_path_through_link(startPose, linkEnd, linkStart, targetNode);

    for (const auto &[path, distance] : {result1, result2}) {
      if (distance && (!bestDistance || *distance < *bestDistance)) {
        bestPath     = path;
        bestDistance = distance;
      }
    }
  }

  if (!bestPath) {
    return Base::dijkstra_from_pose(startPose, targetNode);
  }

  return {*bestPath, *bestDistance};
}

std::pair<std::vector<v_edge_t>, std::optional<double>>
LinkBasedPlanner::dijkstra_with_smart_link_following(
    const Eigen::Vector2d &startPose, const v_node_t &targetNode,
    size_t maxLinks, double adaptiveWeight) const {
  if (!Base::has_node(targetNode)) {
    return {{}, std::nullopt};
  }

  auto closestLinks = Base::get_closest_links(startPose, maxLinks);
  if (closestLinks.empty()) {
    return Base::dijkstra_from_pose(startPose, targetNode);
  }

  std::optional<std::vector<PathSegment>> bestPath;
  std::optional<double>                   bestScore;

  for (const auto &[linkStart, linkEnd, linkDistance] : closestLinks) {
    auto [path, distance] =
        build_path_through_link(startPose, linkStart, linkEnd, targetNode);
    if (!distance)
      continue;

    // Calculate adaptive score
    double score = linkDistance * adaptiveWeight + *distance;

    if (!bestScore || score < *bestScore) {
      bestPath  = path;
      bestScore = score;
    }
  }

  if (!bestPath) {
    return Base::dijkstra_from_pose(startPose, targetNode);
  }

  double totalDistance = Base::calculate_path_distance(*bestPath);
  return {*bestPath, totalDistance};
}

std::pair<std::vector<v_edge_t>, std::optional<double>>
LinkBasedPlanner::build_path_through_link(const Eigen::Vector2d &startPose,
                                          const v_node_t        &linkStart,
                                          const v_node_t        &linkEnd,
                                          const v_node_t &targetNode) const {
  // Path from exit node to target
  auto [exitToTarget, exitToTargetDist] = Base::dijkstra(linkEnd, targetNode);
  if (!exitToTargetDist) {
    return {{}, std::nullopt};
  }

  // Build complete path
  Eigen::Vector2d projectionPoint =
      Base::get_projection_point_on_line_segment(startPose, linkStart, linkEnd);
  const Eigen::Vector2d &accessPose = Base::get_pose(linkStart);
  const Eigen::Vector2d &exitPose   = Base::get_pose(linkEnd);

  std::vector<v_edge_t> completePath = {
      create_virtual_edge(startPose, projectionPoint),  // Approach to link
      create_virtual_edge(projectionPoint, accessPose), // Move to link start
      create_virtual_edge(accessPose, exitPose),        // Follow link topology
  };
  completePath.insert(completePath.end(), exitToTarget.begin(),
                      exitToTarget.end());

  double totalDistance = Base::calculate_path_distance(completePath);
  return {completePath, totalDistance};
}

} // namespace vrobot_route