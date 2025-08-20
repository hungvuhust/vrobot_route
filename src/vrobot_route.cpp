#include "rclcpp/rclcpp.hpp"

#include <vrobot_route/graph_pose.hpp>

#include <nav_msgs/msg/path.hpp>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // Node ros2
  auto node      = rclcpp::Node::make_shared("vrobot_route");
  // publisher
  auto publisher = node->create_publisher<nav_msgs::msg::Path>(
    "vrobot_route/path", rclcpp::QoS(10).transient_local());

  auto db_manager = std::make_shared<vrobot_route::DbManager>(
    "host=127.0.0.1 port=5432 dbname=amr_01 user=amr password=1234512345");

  auto map           = db_manager->getMap("mapmoi");
  auto nodes         = db_manager->getNodes("mapmoi");
  auto straightlinks = db_manager->getStraightlinks("mapmoi");
  auto curvelinks    = db_manager->getCurvelinks("mapmoi");

  std::vector<vrobot_route::v_node_t>             vnodes;
  std::unordered_map<int, vrobot_route::v_node_t> vnodes_map;
  std::cout << "Nodes: " << nodes.size() << std::endl;
  for (const auto &node : nodes) {
    vrobot_route::v_node_t vnode;
    vnode.id_    = node.getValueOfId();
    vnode.pose_  = Eigen::Vector2d(node.getValueOfX(), node.getValueOfY());
    vnode.theta_ = node.getValueOfTheta();

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

    vedges.push_back(vedge);
  }
  std::cout << "Curvelinks: " << curvelinks.size() << std::endl;
  for (const auto &curvelink : curvelinks) {
    vrobot_route::v_edge_t vedge;
    vedge.id_         = curvelink.getValueOfIdCurveLink();
    vedge.start_node_ = vnodes_map[curvelink.getValueOfIdStart()];
    vedge.end_node_   = vnodes_map[curvelink.getValueOfIdEnd()];
    vedge.type_       = vrobot_route::v_link_type_t::CURVE;
    vedge.length_     = (vnodes_map[curvelink.getValueOfIdStart()].pose_ -
                     vnodes_map[curvelink.getValueOfIdEnd()].pose_)
                      .norm();

    vedge.control_points_.push_back(
      Eigen::Vector2d(curvelink.getValueOfControlPoint1X(),
                      curvelink.getValueOfControlPoint1Y()));
    vedge.control_points_.push_back(
      Eigen::Vector2d(curvelink.getValueOfControlPoint2X(),
                      curvelink.getValueOfControlPoint2Y()));

    vedge.max_vel_ = curvelink.getValueOfMaxVelocity();
    vedge.width_   = 0.5;

    vedges.push_back(vedge);
  }

  vrobot_route::GraphPose graph(vnodes, vedges);

  vrobot_route::GraphPose::PlanningConfig config;
  config.directThreshold = 0.3;
  config.maxLinkDistance = 0.5;
  config.enablePruning   = false;

  auto path = graph.planPath(Eigen::Vector2d(1.75, 0.5), vnodes[7], config);

  if (path.totalDistance.has_value()) {
    for (const auto &segment : path.pathSegments) {
      std::cout << "\t- Segment: " << segment.id_ << "\t"
                << "Start: " << segment.start_node_.id_ << "\t"
                << "End: " << segment.end_node_.id_ << "\t"
                << "Length: " << segment.length_ << std::endl;
    }

    auto vpath =
      graph.pathSegmentsToVPath(path.pathSegments, "map", node->now(), 0.01);
    auto nav_path = graph.toPath(vpath);

    std::cout << "Publishing path with " << nav_path.poses.size() << " poses"
              << std::endl;
    publisher->publish(nav_path);
    std::cout << "\t- Distance: " << path.totalDistance.value() << std::endl;
  } else {
    std::cout << "No path found!" << std::endl;
  }

  // spin
  rclcpp::spin(node);

  // shutdown
  rclcpp::shutdown();

  return 0;
}