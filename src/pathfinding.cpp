#include <vrobot_route/pathfinding.hpp>

namespace vrobot_route {

std::pair<std::vector<Pathfinding::PathSegment>, std::optional<double>>
Pathfinding::dijkstra(const v_node_t &start_node,
                      const v_node_t &target_node) const {
  // Check if start and target nodes exist
  if (!has_node(start_node) || !has_node(target_node)) {
    std::cout << "Start or target node not found" << std::endl;
    return std::make_pair(std::vector<PathSegment>(), std::nullopt);
  }

  if (start_node == target_node) {
    std::cout << "Start and target node are the same" << std::endl;
    return std::make_pair(std::vector<PathSegment>(), 0.0);
  }

  // Initialize distances and predecessors
  struct NodeInfo {
    double                  distance = std::numeric_limits<double>::infinity();
    std::optional<v_node_t> predecessor = std::nullopt;
  };

  std::unordered_map<v_node_t, NodeInfo, VNodeHash> node_info;
  for (const auto &node : Base::nodes_) {
    node_info[node] = NodeInfo();
  }
  node_info[start_node].distance = 0.0;

  // Priority queue for Dijkstra
  using PQItem = std::pair<double, v_node_t>;
  std::priority_queue<PQItem, std::vector<PQItem>, std::greater<>> pq;
  pq.emplace(0.0, start_node);

  while (!pq.empty()) {
    auto [current_dist, current_node] = pq.top();
    pq.pop();

    // Skip if we've already processed this node
    if (current_dist > node_info[current_node].distance) {
      continue;
    }

    // Check if we've reached the target node
    if (current_node == target_node) {
      break;
    }

    // Process neighbors
    for (const auto &[neighbor, edge] : Base::get_neighbors(current_node)) {
      double new_dist = node_info[current_node].distance + edge.length_;
      if (new_dist < node_info[neighbor].distance) {
        node_info[neighbor].distance    = new_dist;
        node_info[neighbor].predecessor = current_node;
        pq.emplace(new_dist, neighbor);
      }
    }
  }

  // Check if path was found
  if (!node_info[target_node].predecessor) {
    std::cout << "No path found" << std::endl;
    return std::make_pair(std::vector<PathSegment>(), std::nullopt);
  }

  // Reconstruct path
  std::vector<PathSegment> path;
  v_node_t                 current = target_node;
  while (current != start_node) {
    if (!node_info[current].predecessor) {
      return std::make_pair(std::vector<PathSegment>(), std::nullopt);
    }
    v_node_t prev  = node_info[current].predecessor.value();
    // Find edge between prev and current
    bool     found = false;
    v_edge_t edge_found;
    for (const auto &[neighbor, edge] : Base::get_neighbors(prev)) {
      if (neighbor == current) {
        edge_found = edge;
        found      = true;
        break;
      }
    }
    if (!found) {
      std::cout << "Edge not found between nodes " << prev.id_ << " and "
                << current.id_ << std::endl;
      return std::make_pair(std::vector<PathSegment>(), std::nullopt);
    }
    path.push_back(edge_found);
    current = prev;
  }
  std::reverse(path.begin(), path.end());
  return std::make_pair(path, node_info[target_node].distance);
}

std::pair<std::vector<Pathfinding::PathSegment>, std::optional<double>>
Pathfinding::dijkstra_from_pose(const Eigen::Vector2d &start_pose,
                                const v_node_t        &target_node,
                                double max_distance_to_graph) const {
  if (!Base::has_node(target_node)) {
    return {{}, std::nullopt};
  }

  // Find closest node to start pose
  v_node_t closest_node = Base::get_nearest_node(start_pose);

  // Check if distance to closest node exceeds threshold
  double distance_to_closest = Base::get_distance_to_nearest_node(start_pose);
  if (distance_to_closest > max_distance_to_graph) {
    return {{}, std::nullopt};  // Pose is too far from graph
  }

  // Get path from closest node to target
  auto [nodePath, nodeDistance] = dijkstra(closest_node, target_node);

  if (!nodeDistance) {
    return {{}, std::nullopt};
  }

  // Add initial segment from start pose to closest node
  std::vector<PathSegment> fullPath;
  const Eigen::Vector2d   &closest_pose = Base::get_pose(closest_node);
  double initial_distance               = (start_pose - closest_pose).norm();

  if (initial_distance > 1e-6) {  // Only add if not already at the node
    fullPath.emplace_back(create_virtual_edge(start_pose, closest_pose));
  }

  // Add the rest of the path
  fullPath.insert(fullPath.end(), nodePath.begin(), nodePath.end());

  double total_distance = initial_distance + *nodeDistance;
  return {fullPath, total_distance};
}

std::pair<std::vector<Pathfinding::PathSegment>, std::optional<double>>
Pathfinding::dijkstra_pose_to_pose(const Eigen::Vector2d &start_pose,
                                   const Eigen::Vector2d &target_pose,
                                   double maxDistanceToGraph) const {
  if (Base::is_empty()) {
    return {{}, std::nullopt};
  }

  // Find closest nodes
  v_node_t start_node  = Base::get_nearest_node(start_pose);
  v_node_t target_node = Base::get_nearest_node(target_pose);

  // Check if start pose is within threshold
  double start_distance_to_graph =
    (start_pose - Base::get_pose(start_node)).norm();
  if (start_distance_to_graph > maxDistanceToGraph) {
    return {{}, std::nullopt};  // Start pose is too far from graph
  }

  // Check if target pose is within threshold
  double target_distance_to_graph =
    (target_pose - Base::get_pose(target_node)).norm();
  if (target_distance_to_graph > maxDistanceToGraph) {
    return {{}, std::nullopt};  // Target pose is too far from graph
  }

  // Get path between nodes
  auto [nodePath, nodeDistance] = dijkstra(start_node, target_node);

  if (!nodeDistance) {
    return {{}, std::nullopt};
  }

  // Build complete path
  std::vector<PathSegment> fullPath;
  double                   totalDistance = 0.0;

  // Start to first node
  const Eigen::Vector2d &start_node_pose = Base::get_pose(start_node);
  double                 start_distance = (start_pose - start_node_pose).norm();
  if (start_distance > 1e-6) {
    v_edge_t virtual_edge = create_virtual_edge(start_pose, start_node_pose);
    fullPath.emplace_back(virtual_edge);
    totalDistance += start_distance;
  }

  // Node-to-node path
  fullPath.insert(fullPath.end(), nodePath.begin(), nodePath.end());
  totalDistance += *nodeDistance;

  // Last node to target
  const Eigen::Vector2d &target_node_pose = Base::get_pose(target_node);
  double                 end_distance = (target_node_pose - target_pose).norm();
  if (end_distance > 1e-6) {
    v_edge_t virtual_edge = create_virtual_edge(target_node_pose, target_pose);

    fullPath.emplace_back(virtual_edge);
    totalDistance += end_distance;
  }

  return {fullPath, totalDistance};
}

double Pathfinding::calculate_path_distance(
  const std::vector<Pathfinding::PathSegment> &path_segments) const {
  double distance = 0.0;
  for (const auto &segment : path_segments) {
    distance += segment.length_;
  }
  return distance;
}

bool Pathfinding::is_valid_path(
  const std::vector<Pathfinding::PathSegment> &path_segments) const {
  for (const auto &segment : path_segments) {
    if (segment.start_node_ == segment.end_node_) {
      return false;
    }
  }
  return true;
}

}  // namespace vrobot_route