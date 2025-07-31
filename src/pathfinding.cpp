#include <vrobot_route/pathfinding.hpp>

namespace vrobot_route {

std::pair<std::vector<Pathfinding::PathSegment>, std::optional<double>>
Pathfinding::dijkstra(const v_node_t &start_node, const v_node_t &target_node) {
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
  auto cmp = [](const PQItem &a, const PQItem &b) { return a.first > b.first; };
  std::priority_queue<PQItem, std::vector<PQItem>, decltype(cmp)> pq(cmp);
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

  if (node_info[target_node].predecessor) {
    std::cout << "Predecessor: "
              << node_info[target_node].predecessor.value().id_ << std::endl;
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
    v_node_t prev = node_info[current].predecessor.value();
    // Find edge between prev and current
    v_edge_t edge_found;
    for (const auto &[neighbor, edge] : Base::get_neighbors(prev)) {
      if (neighbor == current) {
        edge_found = edge;
        break;
      }
    }
    path.push_back(edge_found);
    current = prev;
  }
  std::reverse(path.begin(), path.end());
  return std::make_pair(path, node_info[target_node].distance);
}

} // namespace vrobot_route