#include "rclcpp/rclcpp.hpp"

#include <vrobot_route/datastructor.hpp>
#include <vrobot_route/db_client.hpp>
#include <vrobot_route/geometric_utils.hpp>
#include <vrobot_route/pathfinding.hpp>

class GraphPose : public virtual vrobot_route::VGraph,
                  public virtual vrobot_route::GeometricUtils,
                  public virtual vrobot_route::Pathfinding {
public:
  using Base        = vrobot_route::VGraph;
  using PathSegment = vrobot_route::v_edge_t;

  GraphPose() = default;

  GraphPose(const std::vector<vrobot_route::v_node_t> &vnodes,
            const std::vector<vrobot_route::v_edge_t> &vedges)
      : vrobot_route::VGraph(vnodes, vedges) {}
};
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto db_manager = std::make_shared<vrobot_route::DbManager>(
      "host=127.0.0.1 port=5432 dbname=amr_01 user=amr password=1234512345");

  auto map           = db_manager->getMap("mapmoi");
  auto nodes         = db_manager->getNodes("mapmoi");
  auto straightlinks = db_manager->getStraightlinks("mapmoi");
  auto curvelinks    = db_manager->getCurvelinks("mapmoi");

  std::vector<vrobot_route::v_node_t> vnodes;
  for (const auto &node : nodes) {
    vrobot_route::v_node_t vnode;
    vnode.id_    = node.getValueOfId();
    vnode.pose_  = Eigen::Vector2d(node.getValueOfX(), node.getValueOfY());
    vnode.theta_ = node.getValueOfTheta();

    std::cout << "Node: " << vnode.id_ << " Pose: " << vnode.pose_.transpose()
              << " Theta: " << vnode.theta_ << std::endl;
    vnodes.push_back(vnode);
  }

  std::vector<vrobot_route::v_edge_t> vedges;
  for (const auto &straightlink : straightlinks) {
    vrobot_route::v_edge_t vedge;
    vedge.id_         = straightlink.getValueOfIdStraightLink();
    vedge.start_node_ = vnodes[straightlink.getValueOfIdStart()];
    vedge.end_node_   = vnodes[straightlink.getValueOfIdEnd()];
    vedge.type_       = vrobot_route::v_link_type_t::STRAIGHT;
    vedge.length_     = (vnodes[straightlink.getValueOfIdStart()].pose_ -
                     vnodes[straightlink.getValueOfIdEnd()].pose_)
                        .norm();
    vedge.width_   = 0.5;
    vedge.max_vel_ = straightlink.getValueOfMaxVelocity();

    std::cout << "Edge: " << vedge.id_ << " Start: " << vedge.start_node_.id_
              << " End: " << vedge.end_node_.id_ << " Length: " << vedge.length_
              << " Width: " << vedge.width_ << " Max Vel: " << vedge.max_vel_
              << std::endl;
    vedges.push_back(vedge);
  }

  GraphPose graph(vnodes, vedges);

  auto path = graph.dijkstra(vnodes[0], vnodes[1]);

  if (path.second.has_value()) {
    for (const auto &segment : path.first) {
      std::cout << "Segment: " << segment.id_ << std::endl;
    }
    std::cout << "Distance: " << path.second.value() << std::endl;
  } else {
    std::cout << "No path found!" << std::endl;
  }

  rclcpp::shutdown();

  return 0;
}