#include "rclcpp/rclcpp.hpp"

#include <vrobot_route/graph_pose.hpp>

#include <nav_msgs/msg/path.hpp>

#include <vrobot_route/vrobot_route.hpp>
#include <vrobot_route/benzier.hpp>

namespace vrobot_route {

VrobotRoute::VrobotRoute() : rclcpp::Node("vrobot_route") {
  init_db();
  init_publishers();
}

VrobotRoute::~VrobotRoute() {
}

void VrobotRoute::init_db() {
  db_manager_ = std::make_shared<vrobot_route::DbManager>(connection_info_);
}

void VrobotRoute::init_publishers() {
  graph_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/graph_vis", rclcpp::QoS(10).transient_local());

  path_publisher_ =
    create_publisher<nav_msgs::msg::Path>("vrobot_route/path",
                                          rclcpp::QoS(10).transient_local());
}

bool VrobotRoute::update_graph(std::string map_name) {
  try {
    if (!db_manager_) {
      RCLCPP_ERROR(get_logger(), "Database manager not initialized");
      return false;
    }

    map_name_ = map_name;

    auto map           = db_manager_->getMap(map_name_);
    auto nodes         = db_manager_->getNodes(map_name_);
    auto straightlinks = db_manager_->getStraightlinks(map_name_);
    auto curvelinks    = db_manager_->getCurvelinks(map_name_);

    vnodes_.clear();
    vedges_.clear();

    std::unordered_map<int, vrobot_route::v_node_t> vnodes_map;
    RCLCPP_INFO(get_logger(), "Nodes: %ld", nodes.size());
    for (const auto &node : nodes) {
      vrobot_route::v_node_t vnode;
      vnode.id_    = node.getValueOfId();
      vnode.pose_  = Eigen::Vector2d(node.getValueOfX(), node.getValueOfY());
      vnode.theta_ = node.getValueOfTheta();

      vnodes_.push_back(vnode);
      vnodes_map[vnode.id_] = vnode;
    }

    RCLCPP_INFO(get_logger(), "Straightlinks: %ld", straightlinks.size());
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

      vedges_.push_back(vedge);
    }
    RCLCPP_INFO(get_logger(), "Curvelinks: %ld", curvelinks.size());
    for (const auto &curvelink : curvelinks) {
      vrobot_route::v_edge_t vedge;
      vedge.id_         = curvelink.getValueOfIdCurveLink();
      vedge.start_node_ = vnodes_map[curvelink.getValueOfIdStart()];
      vedge.end_node_   = vnodes_map[curvelink.getValueOfIdEnd()];
      vedge.type_       = vrobot_route::v_link_type_t::CURVE;
      // Tính length thực của curve thay vì khoảng cách thẳng
      bezier::Bezier<3> bezierCurve(
        {bezier::Point(vnodes_map[curvelink.getValueOfIdStart()].pose_.x(),
                       vnodes_map[curvelink.getValueOfIdStart()].pose_.y()),
         bezier::Point(curvelink.getValueOfControlPoint1X(),
                       curvelink.getValueOfControlPoint1Y()),
         bezier::Point(curvelink.getValueOfControlPoint2X(),
                       curvelink.getValueOfControlPoint2Y()),
         bezier::Point(vnodes_map[curvelink.getValueOfIdEnd()].pose_.x(),
                       vnodes_map[curvelink.getValueOfIdEnd()].pose_.y())});
      
      // Tính arc length bằng cách sample curve
      double arcLength = 0.0;
      const int samples = 100;
      bezier::Point prevPoint = bezierCurve.valueAt(0.0);
      
      for (int i = 1; i <= samples; ++i) {
        double t = static_cast<double>(i) / samples;
        bezier::Point currentPoint = bezierCurve.valueAt(t);
        double dx = currentPoint.x - prevPoint.x;
        double dy = currentPoint.y - prevPoint.y;
        arcLength += std::sqrt(dx * dx + dy * dy);
        prevPoint = currentPoint;
      }
      
      vedge.length_ = arcLength;

      vedge.control_points_.push_back(
        Eigen::Vector2d(curvelink.getValueOfControlPoint1X(),
                        curvelink.getValueOfControlPoint1Y()));
      vedge.control_points_.push_back(
        Eigen::Vector2d(curvelink.getValueOfControlPoint2X(),
                        curvelink.getValueOfControlPoint2Y()));

      vedge.max_vel_ = curvelink.getValueOfMaxVelocity();
      vedge.width_   = 0.5;

      vedges_.push_back(vedge);
    }

    graph_ = std::make_shared<vrobot_route::GraphPose>(vnodes_, vedges_);

    auto graph_vis = graph_->visualizeGraph(
      "map", this->get_clock()->now(), 0.1, 0.1, true, true);
    graph_publisher_->publish(graph_vis);

  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_logger(), "Error updating graph: %s", e.what());
    return false;
  }
  return true;
}

vrobot_route::GraphPose::PlanningResult VrobotRoute::test(
  Eigen::Vector2d start,
  int             end_id,
  std::string     map_name) {
  vrobot_route::GraphPose::PlanningConfig config;
  config.directThreshold = 0.3;
  config.maxLinkDistance = 0.5;
  config.enablePruning   = true;

  if (!update_graph(map_name)) {
    RCLCPP_ERROR(get_logger(), "Failed to update graph");
    return vrobot_route::GraphPose::PlanningResult();
  }

  vrobot_route::v_node_t end_node;
  for (const auto &vnode : vnodes_) {
    if (vnode.id_ == end_id) {
      end_node = vnode;
      break;
    }
  }

  return graph_->planPath(start, end_node, config);
}

nav_msgs::msg::Path VrobotRoute::toPath(
  const std::vector<vrobot_route::v_edge_t> &path_segments,
  const std::string                         &frame_id,
  const rclcpp::Time                        &timestamp,
  double                                     resolution) {
  auto vpath =
    graph_->pathSegmentsToVPath(path_segments, frame_id, timestamp, resolution);
  return graph_->toPath(vpath);
}

void VrobotRoute::publish_path(const nav_msgs::msg::Path &path) {
  path_publisher_->publish(path);
}

}  // namespace vrobot_route

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // Node ros2
  auto node = std::make_shared<vrobot_route::VrobotRoute>();

  auto path = node->test(Eigen::Vector2d(0.5, -0.2), 8, "mapmoi");

  if (path.totalDistance.has_value()) {
    for (const auto &segment : path.pathSegments) {
      std::cout << "\t- Segment: " << segment.id_ << "\t"
                << "Start: " << segment.start_node_.id_ << "\t"
                << "End: " << segment.end_node_.id_ << "\t"
                << "Length: " << segment.length_ << std::endl;
    }

    auto nav_path = node->toPath(path.pathSegments, "map", node->now(), 0.01);

    std::cout << "Publishing path with " << nav_path.poses.size() << " poses"
              << std::endl;
    node->publish_path(nav_path);
    std::cout << "\t- Distance: " << path.totalDistance.value() << std::endl;
    std::cout << "\t- Algorithm used: " << path.algorithmUsed << std::endl;
    std::cout << "\t- Path length: " << path.pathSegments.size() << std::endl;
  } else {
    std::cout << "No path found!" << std::endl;
  }

  // spin
  rclcpp::spin(node);

  // shutdown
  rclcpp::shutdown();

  return 0;
}