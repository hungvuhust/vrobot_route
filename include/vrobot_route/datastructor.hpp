#pragma once

#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <unordered_map>
#include <vrobot_route/db_client.hpp>

namespace vrobot_route {

typedef enum VLinkType {
  STRAIGHT,
  CURVE,
} v_link_type_t;

typedef struct VNode {
  int             id_;
  std::string     name_;
  Eigen::Vector2d pose_;
  double          theta_;
  double          curvature_;

  // Equality operator for unordered_map
  bool operator==(const VNode &other) const {
    return id_ == other.id_;
  }
  bool operator!=(const VNode &other) const {
    return id_ != other.id_;
  }
  bool operator<(const VNode &other) const {
    return id_ < other.id_;
  }

  using Ptr = std::shared_ptr<VNode>;
} v_node_t;

typedef struct VEdge {
  int                          id_;
  std::string                  name_;
  v_node_t                     start_node_;
  v_node_t                     end_node_;
  std::vector<Eigen::Vector2d> control_points_;
  v_link_type_t                type_;
  double                       length_;
  double                       width_;
  double                       max_vel_;

  using Ptr = std::shared_ptr<VEdge>;
} v_edge_t;

inline v_edge_t create_virtual_edge(const Eigen::Vector2d &start_pose,
                                    const Eigen::Vector2d &end_pose) {
  v_edge_t edge;
  v_node_t start_node, end_node;

  start_node.id_    = -1;
  start_node.name_  = "virtual_start_node";
  start_node.pose_  = start_pose;
  start_node.theta_ = 0.0;

  end_node.id_    = -2;
  end_node.name_  = "virtual_end_node";
  end_node.pose_  = end_pose;
  end_node.theta_ = 0.0;

  edge.start_node_ = start_node;
  edge.end_node_   = end_node;

  edge.id_      = -1;
  edge.name_    = "virtual_edge";
  edge.type_    = v_link_type_t::STRAIGHT;
  edge.length_  = (start_pose - end_pose).norm();
  edge.width_   = 0.5;
  edge.max_vel_ = 0.3;

  return edge;
}

// Hash function for VNode
struct VNodeHash {
  std::size_t operator()(const v_node_t &node) const {
    return std::hash<int>()(node.id_);
  }
};

// Hash function for VEdge
struct VEdgeHash {
  std::size_t operator()(const v_edge_t &edge) const {
    return std::hash<int>()(edge.start_node_.id_) ^
           std::hash<int>()(edge.end_node_.id_);
  }
};

struct VGraph {
  using VPair = std::pair<v_node_t, v_edge_t>;

  std::vector<v_node_t>                                       nodes_;
  std::vector<v_edge_t>                                       edges_;
  std::unordered_map<v_node_t, std::vector<VPair>, VNodeHash> adj_list_;

  VGraph()                          = default;
  VGraph(const VGraph &)            = default;
  VGraph &operator=(const VGraph &) = default;
  virtual ~VGraph()                 = default;

  explicit VGraph(const std::vector<v_node_t> &nodes,
                  const std::vector<v_edge_t> &edges)
    : nodes_(nodes), edges_(edges) {
    build_graph(nodes, edges);
  }

  // ========================================================================
  // BASIC ACCESSORS
  // ========================================================================

  /**
   * @brief Get pose of a node
   * @param nodeId Node identifier
   * @return Pose of the node
   */
  inline Eigen::Vector2d get_pose(const v_node_t &node) const {
    auto it = std::find(nodes_.begin(), nodes_.end(), node);
    if (it == nodes_.end()) {
      throw std::runtime_error("Node not found");
    }
    return it->pose_;
  }

  /**
   * @brief Check if node exists in graph
   * @param nodeId Node identifier
   * @return True if node exists
   */
  inline bool has_node(const v_node_t &node) const {
    return std::find(nodes_.begin(), nodes_.end(), node) != nodes_.end();
  }

  /**
   * @brief Get all node IDs
   * @return Vector of all node IDs
   */
  inline std::vector<int> get_node_ids() const {
    std::vector<int> ids;
    for (const auto &node : nodes_) {
      ids.push_back(node.id_);
    }
    return ids;
  }

  /**
   * @brief Get neighbors of a node
   * @param nodeId Node identifier
   * @return Vector of (neighbor_id, edge) pairs
   */
  inline std::vector<std::pair<v_node_t, v_edge_t>> get_neighbors(
    const v_node_t &node) const {
    auto it = adj_list_.find(node);
    if (it == adj_list_.end()) {
      static const std::vector<std::pair<v_node_t, v_edge_t>> empty;
      return empty;
    }
    return it->second;
  }

  /**
   * @brief Get number of nodes
   * @return Number of nodes in graph
   */
  inline size_t get_num_nodes() const {
    return nodes_.size();
  }

  /**
   * @brief Get number of edges
   * @return Total number of directed edges
   */
  inline size_t get_num_edges() const {
    return edges_.size();
  }

  /**
   * @brief Check if graph is empty
   * @return True if graph has no nodes
   */
  inline bool is_empty() const {
    return nodes_.empty();
  }

  /**
   * @brief Clear all data
   */
  inline void clear() {
    nodes_.clear();
    edges_.clear();
    adj_list_.clear();
  }

private:
  inline void build_graph(const std::vector<v_node_t> &nodes,
                          const std::vector<v_edge_t> &edges) {
    for (const auto &node : nodes) {
      adj_list_[node] = {};
    }
    for (const auto &edge : edges) {
      adj_list_[edge.start_node_].emplace_back(edge.end_node_, edge);
    }

#ifdef DEBUG
    // Log all adj_list_
    for (const auto &[node, edges] : adj_list_) {
      std::cout << "Node: " << node.id_ << std::endl;
      for (const auto &[neighbor, edge] : edges) {
        std::cout << "  Neighbor: " << neighbor.id_ << "----" << edge.id_
                  << std::endl;
      }
    }
#endif
  }
};

}  // namespace vrobot_route