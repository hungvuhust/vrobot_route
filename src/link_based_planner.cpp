#include "vrobot_route/datastructor.hpp"
#include <vrobot_route/link_based_planner.hpp>

namespace vrobot_route {

std::pair<std::vector<v_edge_t>, std::optional<double>> LinkBasedPlanner::
  dijkstra_with_modular_link_approach(const Eigen::Vector2d &startPose,
                                      const v_node_t        &targetNode,
                                      double                 directThreshold,
                                      size_t                 maxLinks,
                                      double                 linkDistanceWeight,
                                      double                 maxLinkDistance,
                                      double graphDistanceWeight) const {
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

  std::cout << "Closest links: " << closestLinks.size() << std::endl;
  for (const auto &[linkStart, linkEnd, linkDistance] : closestLinks) {
    std::cout << "Link: " << linkStart.id_ << " -> " << linkEnd.id_
              << " (Distance: " << linkDistance << ")" << std::endl;
  }

  // Step 2: Evaluate each link using weighted scoring
  std::optional<std::vector<PathSegment>> bestPath;
  std::optional<double>                   bestScore;

  for (const auto &[linkStart, linkEnd, linkDistance] : closestLinks) {
    // Calculate path from link to destination
    double linkToTargetDist = 0.0; // Default for when linkEnd == targetNode
    std::vector<PathSegment> linkToTarget; // Empty when linkEnd == targetNode
    
    if (linkEnd != targetNode && linkStart != targetNode) {
      auto [path, dist] = Base::dijkstra(linkEnd, targetNode);
      if (!dist) {
        continue;
      }
      linkToTarget = path;
      linkToTargetDist = *dist;
    }

    // Calculate weighted score
    double score = this->calculate_weighted_score(linkDistance,
                                                  linkToTargetDist,
                                                  linkDistanceWeight,
                                                  graphDistanceWeight);

    if (!bestScore || score < *bestScore) {
      // Step 3: Build complete path without backtracking
      Eigen::Vector2d projectionPoint =
        Base::get_smart_projection_point(startPose, linkStart, linkEnd);
      const Eigen::Vector2d &linkEndPose = Base::get_pose(linkEnd);

      std::vector<PathSegment> candidatePath;
      candidatePath.emplace_back(
        create_virtual_edge(startPose, projectionPoint));
      
      // Chỉ thêm edge đến linkEnd nếu projection point không trùng với linkEnd
      if ((projectionPoint - linkEndPose).norm() > 0.01) {
        // Kiểm tra xem có phải đường cong không để xử lý đúng
        const v_edge_t *edge = Base::find_edge_between_nodes(linkStart, linkEnd);
        if (edge && edge->type_ == v_link_type_t::CURVE) {
          // Giữ nguyên edge gốc từ database, chỉ thêm metadata về parameter
          auto [proj_point_precise, t] = Base::get_projection_point_on_curve(startPose, *edge);
          
          v_edge_t curve_segment = *edge; // Copy original edge from database
          curve_segment.curve_start_param_ = t;      // Set start parameter
          curve_segment.curve_end_param_ = 1.0;      // End at original curve end
          curve_segment.is_partial_curve_ = true;    // Mark as partial for rendering
          
          candidatePath.emplace_back(curve_segment);
        } else {
          candidatePath.emplace_back(
            create_virtual_edge(projectionPoint, linkEndPose));
        }
      }
      
      candidatePath.insert(candidatePath.end(),
                           linkToTarget.begin(),
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

std::pair<std::vector<v_edge_t>, std::optional<double>> LinkBasedPlanner::
  dijkstra_with_greedy_strategy(const Eigen::Vector2d &startPose,
                                const v_node_t        &targetNode,
                                double                 tolerance,
                                size_t                 maxLinks) const {
  if (!Base::has_node(targetNode)) {
    return {{}, std::nullopt};
  }

  // Tìm tối đa maxLinks đường gần nhất trong tolerance
  auto closestLinks = Base::get_closest_links(startPose, maxLinks, tolerance);
  if (closestLinks.empty()) {
    // Fallback to standard dijkstra if no links within tolerance
    return Base::dijkstra_from_pose(startPose, targetNode);
  }

  std::cout << "Greedy strategy: evaluating " << closestLinks.size() << " links within tolerance " << tolerance << std::endl;

  std::optional<std::vector<PathSegment>> bestPath;
  std::optional<double>                   bestTotalDistance;

  for (const auto &[linkStart, linkEnd, linkDistance] : closestLinks) {
    std::cout << "Evaluating link: " << linkStart.id_ << " -> " << linkEnd.id_ << " (distance: " << linkDistance << ")" << std::endl;

    // Xây dựng path hoàn chỉnh cho link này
    auto [candidatePath, candidateDistance] = 
      build_path_through_link(startPose, linkStart, linkEnd, targetNode);
    
    if (!candidateDistance) {
      std::cout << "  No valid path found through this link" << std::endl;
      continue;
    }

    std::cout << "  Complete path distance: " << *candidateDistance << std::endl;

    // So sánh với path tốt nhất hiện tại
    if (!bestTotalDistance || *candidateDistance < *bestTotalDistance) {
      bestPath = candidatePath;
      bestTotalDistance = candidateDistance;
      std::cout << "  New best path! Total distance: " << *candidateDistance << std::endl;
    }
  }

  if (!bestPath) {
    std::cout << "No valid path found through any link, falling back to standard dijkstra" << std::endl;
    return Base::dijkstra_from_pose(startPose, targetNode);
  }

  std::cout << "Greedy strategy result: best total distance = " << *bestTotalDistance << std::endl;
  return {*bestPath, *bestTotalDistance};
}

std::pair<std::vector<v_edge_t>, std::optional<double>> LinkBasedPlanner::
  dijkstra_with_link_access(const Eigen::Vector2d &startPose,
                            const v_node_t        &targetNode,
                            double                 distanceThreshold,
                            size_t                 maxLinks,
                            double                 linkDistanceWeight) const {
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

std::pair<std::vector<v_edge_t>, std::optional<double>> LinkBasedPlanner::
  dijkstra_with_simple_link_access(const Eigen::Vector2d &startPose,
                                   const v_node_t        &targetNode,
                                   size_t                 maxLinks) const {
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
    // Project onto link - sử dụng smart projection cho cả đường thẳng và cong
    Eigen::Vector2d projectionPoint =
      Base::get_smart_projection_point(startPose, linkStart, linkEnd);

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

    // Build complete path - kiểm tra nếu projection point gần access node hơn
    std::vector<PathSegment> candidatePath;
    candidatePath.emplace_back(create_virtual_edge(startPose, projectionPoint));
    
    // Chỉ thêm edge đến access node nếu projection point khác với access node pose
    const Eigen::Vector2d &accessNodePose = Base::get_pose(accessNode);
    if ((projectionPoint - accessNodePose).norm() > 0.01) { // threshold nhỏ để tránh edge quá ngắn
      candidatePath.emplace_back(
        create_virtual_edge(projectionPoint, accessNodePose));
    }

    candidatePath.insert(candidatePath.end(),
                         nodeToTarget.begin(),
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

std::pair<std::vector<v_edge_t>, std::optional<double>> LinkBasedPlanner::
  dijkstra_with_link_following(const Eigen::Vector2d &startPose,
                               const v_node_t        &targetNode,
                               size_t                 maxLinks) const {
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

std::pair<std::vector<v_edge_t>, std::optional<double>> LinkBasedPlanner::
  dijkstra_with_smart_link_following(const Eigen::Vector2d &startPose,
                                     const v_node_t        &targetNode,
                                     size_t                 maxLinks,
                                     double adaptiveWeight) const {
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

std::pair<std::vector<v_edge_t>, std::optional<double>> LinkBasedPlanner::
  build_path_through_link(const Eigen::Vector2d &startPose,
                          const v_node_t        &linkStart,
                          const v_node_t        &linkEnd,
                          const v_node_t        &targetNode) const {
  
  // Build complete path - sử dụng smart projection cho cả đường thẳng và cong
  Eigen::Vector2d projectionPoint =
    Base::get_smart_projection_point(startPose, linkStart, linkEnd);
  const Eigen::Vector2d &exitPose = Base::get_pose(linkEnd);

  // Tạo virtual edge từ projection point đến end node để tiếp tục path
  // không cần backtrack về start node
  const v_edge_t *edge = Base::find_edge_between_nodes(linkStart, linkEnd);
  std::vector<v_edge_t> completePath;
  
  if (edge && edge->type_ == v_link_type_t::CURVE) {
    // Đối với đường cong: giữ nguyên định nghĩa curve gốc, chỉ thêm metadata
    auto [proj_point, t] = Base::get_projection_point_on_curve(startPose, *edge);
    
    // Tạo virtual straight edge từ điểm xuất phát đến projection point
    completePath.emplace_back(create_virtual_edge(startPose, proj_point));
    
    // Giữ nguyên curve gốc từ database, chỉ thêm parameter metadata
    v_edge_t curve_segment = *edge; // Copy original database edge
    curve_segment.curve_start_param_ = t;      // Parameter bắt đầu vẽ
    curve_segment.curve_end_param_ = 1.0;      // Parameter kết thúc vẽ
    curve_segment.is_partial_curve_ = true;    // Flag để rendering system biết
    
    completePath.emplace_back(curve_segment);
  } else {
    // Đối với đường thẳng: đơn giản hơn
    completePath.emplace_back(create_virtual_edge(startPose, projectionPoint));
    if ((projectionPoint - exitPose).norm() > 0.01) {
      completePath.emplace_back(create_virtual_edge(projectionPoint, exitPose));
    }
  }

  // Kiểm tra xem linkEnd hoặc linkStart có phải là target không
  if (linkEnd == targetNode || linkStart == targetNode) {
    // Nếu một trong hai node của link chính là target, không cần thêm path nào nữa
    double totalDistance = Base::calculate_path_distance(completePath);
    return {completePath, totalDistance};
  }

  // Path from exit node to target (chỉ khi linkEnd != targetNode)
  auto [exitToTarget, exitToTargetDist] = Base::dijkstra(linkEnd, targetNode);
  if (!exitToTargetDist) {
    return {{}, std::nullopt};
  }

  completePath.insert(completePath.end(),
                      exitToTarget.begin(),
                      exitToTarget.end());

  double totalDistance = Base::calculate_path_distance(completePath);
  return {completePath, totalDistance};
}

}  // namespace vrobot_route