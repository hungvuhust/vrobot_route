#include "rclcpp/rclcpp.hpp"

#include <vrobot_route/graph_pose.hpp>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto db_manager = std::make_shared<vrobot_route::DbManager>(
      "host=127.0.0.1 port=5432 dbname=amr_01 user=amr password=1234512345");

  auto map           = db_manager->getMap("mapmoi");
  auto nodes         = db_manager->getNodes("mapmoi");
  auto straightlinks = db_manager->getStraightlinks("mapmoi");
  auto curvelinks    = db_manager->getCurvelinks("mapmoi");

  std::vector<vrobot_route::v_node_t>             vnodes;
  std::unordered_map<int, vrobot_route::v_node_t> vnodes_map;
  for (const auto &node : nodes) {
    vrobot_route::v_node_t vnode;
    vnode.id_    = node.getValueOfId();
    vnode.pose_  = Eigen::Vector2d(node.getValueOfX(), node.getValueOfY());
    vnode.theta_ = node.getValueOfTheta();

    std::cout << "Node: " << vnode.id_ << " Pose: " << vnode.pose_.transpose()
              << " Theta: " << vnode.theta_ << std::endl;
    vnodes.push_back(vnode);
    vnodes_map[vnode.id_] = vnode;
  }

  std::vector<vrobot_route::v_edge_t> vedges;

  std::cout << "Straightlinks: " << straightlinks.size() << std::endl;
  for (const auto &straightlink : straightlinks) {
    vrobot_route::v_edge_t vedge;
    vedge.id_         = straightlink.getValueOfIdStraightLink();
    vedge.start_node_ = vnodes_map[straightlink.getValueOfIdStart()];
    vedge.end_node_   = vnodes_map[straightlink.getValueOfIdEnd()];
    vedge.type_       = vrobot_route::v_link_type_t::STRAIGHT;
    vedge.length_     = (vnodes_map[straightlink.getValueOfIdStart()].pose_ -
                     vnodes_map[straightlink.getValueOfIdEnd()].pose_)
                        .norm();
    vedge.width_   = 0.5;
    vedge.max_vel_ = straightlink.getValueOfMaxVelocity();

    std::cout << "Edge: " << vedge.id_ << "\t"
              << "Start: " << vedge.start_node_.id_ << "\t"
              << "End: " << vedge.end_node_.id_ << "\t"
              << "Length: " << vedge.length_ << "\t"
              << "Width: " << vedge.width_ << "\t"
              << "Max Vel: " << vedge.max_vel_ << std::endl;
    vedges.push_back(vedge);
  }

  vrobot_route::GraphPose graph(vnodes, vedges);

  vrobot_route::GraphPose::PlanningConfig config;
  config.directThreshold = 0.3;
  config.maxLinkDistance = 0.5;
  config.enablePruning   = false;

  auto path = graph.planPath(Eigen::Vector2d(0.0, 0.0), vnodes[7], config);

  if (path.totalDistance.has_value()) {
    for (const auto &segment : path.pathSegments) {
      std::cout << "Segment: " << segment.id_ << std::endl;
    }
    std::cout << "Distance: " << path.totalDistance.value() << std::endl;
  } else {
    std::cout << "No path found!" << std::endl;
  }

  rclcpp::shutdown();

  return 0;
}