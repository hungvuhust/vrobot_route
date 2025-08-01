#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vrobot_local_planner/msg/path.hpp>
#include <vrobot_local_planner/msg/planner_pose.hpp>
#include <vrobot_route/datastructor.hpp>

namespace vrobot_route {

class NavConversion : public virtual VGraph {
public:
  using Base = VGraph;

  inline std::vector<Eigen::Vector3d>
  interpolatePoses(const v_edge_t &pathSegment, double resolution) const {

    std::vector<Eigen::Vector3d> interpolatedPoses;

    // Add starting pose
    interpolatedPoses.emplace_back(pathSegment.start_node_.pose_.x(),
                                   pathSegment.start_node_.pose_.y(),
                                   pathSegment.start_node_.theta_);

    const Eigen::Vector3d &start = Eigen::Vector3d(
        pathSegment.start_node_.pose_.x(), pathSegment.start_node_.pose_.y(),
        pathSegment.start_node_.theta_);
    const Eigen::Vector3d &end = Eigen::Vector3d(
        pathSegment.end_node_.pose_.x(), pathSegment.end_node_.pose_.y(),
        pathSegment.end_node_.theta_);

    double dx            = end.x() - start.x();
    double dy            = end.y() - start.y();
    double segmentLength = std::sqrt(dx * dx + dy * dy);

    if (segmentLength < resolution) {
      // Segment too short, just add end point
      interpolatedPoses.push_back(end);
      return interpolatedPoses;
    }

    int numSteps = static_cast<int>(std::ceil(segmentLength / resolution));

    for (int i = 1; i <= numSteps; ++i) {
      double t       = static_cast<double>(i) / numSteps;
      double interpX = start.x() + t * dx;
      double interpY = start.y() + t * dy;

      // Point-to-point orientation (direction of movement)
      double theta = std::atan2(dy, dx);

      interpolatedPoses.emplace_back(interpX, interpY, theta);
    }

    return interpolatedPoses;
  }

  inline vrobot_local_planner::msg::Path
  toVPath(const std::vector<v_edge_t> &pathSegments, const std::string &frameId,
          const rclcpp::Time &timestamp, double resolution = 0.01) const {

    vrobot_local_planner::msg::Path vPath;
    vPath.header.frame_id = frameId;
    vPath.header.stamp    = timestamp;

    for (const auto &segment : pathSegments) {

      // Interpolate poses
      std::vector<Eigen::Vector3d> interpolatedPoses =
          interpolatePoses(segment, resolution);

      for (const auto &pose : interpolatedPoses) {
        vrobot_local_planner::msg::PlannerPose plannerPose;
        plannerPose.header.frame_id = frameId;
        plannerPose.header.stamp    = timestamp;

        plannerPose.pose.position.x = pose.x();
        plannerPose.pose.position.y = pose.y();
        plannerPose.pose.position.z = 0.0;

        // Convert angle to quaternion
        tf2::Quaternion q;
        q.setRPY(0, 0, pose.z());
        plannerPose.pose.orientation = tf2::toMsg(q);

        plannerPose.speed = segment.max_vel_;

        vPath.poses.emplace_back(plannerPose);
      }
    }

    return vPath;
  }
};

} // namespace vrobot_route