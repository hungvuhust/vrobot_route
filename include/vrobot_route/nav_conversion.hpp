#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/detail/marker__struct.hpp>
#include <vrobot_local_planner/msg/path.hpp>
#include <vrobot_local_planner/msg/planner_pose.hpp>
#include <vrobot_route/datastructor.hpp>
#include "vrobot_route/benzier.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;
namespace vrobot_route {

class NavConversion : public virtual VGraph {
public:
  using Base = VGraph;

  inline std::vector<Eigen::Vector3d> interpolatePoses(
    const v_edge_t &pathSegment,
    double          resolution) const {
    std::vector<Eigen::Vector3d> interpolatedPoses;

    if (pathSegment.type_ == v_link_type_t::STRAIGHT) {
      // Add starting pose
      interpolatedPoses.emplace_back(pathSegment.start_node_.pose_.x(),
                                     pathSegment.start_node_.pose_.y(),
                                     pathSegment.start_node_.theta_);

      const Eigen::Vector3d &start =
        Eigen::Vector3d(pathSegment.start_node_.pose_.x(),
                        pathSegment.start_node_.pose_.y(),
                        pathSegment.start_node_.theta_);
      const Eigen::Vector3d &end =
        Eigen::Vector3d(pathSegment.end_node_.pose_.x(),
                        pathSegment.end_node_.pose_.y(),
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
    } else if (pathSegment.type_ == v_link_type_t::CURVE) {
      // Sử dụng parameter metadata để render đúng phần curve cần thiết
      double t_start = pathSegment.is_partial_curve_ ? pathSegment.curve_start_param_ : 0.0;
      double t_end = pathSegment.is_partial_curve_ ? pathSegment.curve_end_param_ : 1.0;
      
      // Tạo Bezier curve từ control points gốc trong database
      bezier::Bezier<3> bezierCurve(
        {bezier::Point(pathSegment.start_node_.pose_.x(),
                       pathSegment.start_node_.pose_.y()),
         bezier::Point(pathSegment.control_points_[0].x(),
                       pathSegment.control_points_[0].y()),
         bezier::Point(pathSegment.control_points_[1].x(),
                       pathSegment.control_points_[1].y()),
         bezier::Point(pathSegment.end_node_.pose_.x(),
                       pathSegment.end_node_.pose_.y())});

      // Sử dụng resolution parameter thay vì hard-code 0.01
      double step = resolution;
      
      // Add starting pose tại t_start
      bezier::Point start_point = bezierCurve.valueAt(t_start);
      interpolatedPoses.emplace_back(start_point.x, start_point.y, pathSegment.start_node_.theta_);

      // Interpolate chỉ trong khoảng [t_start, t_end] của curve gốc
      for (double t = t_start + step; t <= t_end; t += step) {
        if (t >= t_end - step/2) {
          // Điểm cuối: sử dụng t_end chính xác
          bezier::Point end_point = bezierCurve.valueAt(t_end);
          interpolatedPoses.emplace_back(end_point.x, end_point.y, pathSegment.end_node_.theta_);
          break;
        }
        
        bezier::Point point = bezierCurve.valueAt(t);
        bezier::Point nextPoint = bezierCurve.valueAt(std::min(t + step, t_end));
        double theta = std::atan2(nextPoint.y - point.y, nextPoint.x - point.x);

        interpolatedPoses.emplace_back(point.x, point.y, theta);
      }
    }

    return interpolatedPoses;
  }

  inline vrobot_local_planner::msg::Path toVPath(
    const std::vector<v_edge_t> &pathSegments,
    const std::string           &frameId,
    const rclcpp::Time          &timestamp,
    double                       resolution = 0.01) const {
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

  inline nav_msgs::msg::Path toPath(
    const vrobot_local_planner::msg::Path &vPath) const {
    nav_msgs::msg::Path path;
    path.header = vPath.header;

    for (const auto &pose : vPath.poses) {
      geometry_msgs::msg::PoseStamped poseStamped;
      poseStamped.header = vPath.header;
      poseStamped.pose   = pose.pose;
      path.poses.push_back(poseStamped);
    }

    return path;
  }

  // Visualization Graph
  inline MarkerArray visualizeGraph(const std::string  &frameId,
                                    const rclcpp::Time &timestamp,
                                    double              nodeScale,
                                    double              edgeScale,
                                    bool                showNodeLabels,
                                    bool showLinkDirections) const {
    visualization_msgs::msg::MarkerArray markerArray;

    if (Base::adj_list_.empty()) {
      return markerArray;
    }

    // Create node markers
    visualization_msgs::msg::Marker nodeMarker;
    nodeMarker.header.frame_id = frameId;
    nodeMarker.header.stamp    = timestamp;
    nodeMarker.ns              = "graph_nodes";
    nodeMarker.id              = 0;
    nodeMarker.type            = visualization_msgs::msg::Marker::SPHERE_LIST;
    nodeMarker.action          = visualization_msgs::msg::Marker::ADD;
    nodeMarker.scale.x         = nodeScale;
    nodeMarker.scale.y         = nodeScale;
    nodeMarker.scale.z         = nodeScale;
    nodeMarker.color.r         = 0.0;
    nodeMarker.color.g         = 1.0;
    nodeMarker.color.b         = 0.0;
    nodeMarker.color.a         = 1.0;

    // Add node positions
    for (const auto &node : Base::nodes_) {
      geometry_msgs::msg::Point point;
      point.x = node.pose_.x();
      point.y = node.pose_.y();
      point.z = 0.0;
      nodeMarker.points.push_back(point);
    }
    markerArray.markers.push_back(nodeMarker);

    // Create edge markers
    visualization_msgs::msg::Marker edgeMarker;
    edgeMarker.header.frame_id = frameId;
    edgeMarker.header.stamp    = timestamp;
    edgeMarker.ns              = "graph_edges";
    edgeMarker.id              = 1;
    edgeMarker.type            = visualization_msgs::msg::Marker::LINE_LIST;
    edgeMarker.action          = visualization_msgs::msg::Marker::ADD;
    edgeMarker.scale.x         = edgeScale;
    edgeMarker.color.r         = 1.0;
    edgeMarker.color.g         = 0.0;
    edgeMarker.color.b         = 0.0;
    edgeMarker.color.a         = 1.0;

    // Add edge lines
    for (const auto &[fromNode, neighbors] : Base::adj_list_) {
      const auto &fromPose = Base::get_pose(fromNode);

      for (const auto &[toNode, weight] : neighbors) {
        const auto &toPose = Base::get_pose(toNode);

        geometry_msgs::msg::Point fromPoint, toPoint;
        fromPoint.x = fromPose.x();
        fromPoint.y = fromPose.y();
        fromPoint.z = 0.0;

        toPoint.x = toPose.x();
        toPoint.y = toPose.y();
        toPoint.z = 0.0;

        edgeMarker.points.push_back(fromPoint);
        edgeMarker.points.push_back(toPoint);
      }
    }
    markerArray.markers.push_back(edgeMarker);

    // Add node labels if requested
    if (showNodeLabels) {
      int labelId = 100;  // Start label IDs from 100
      for (const auto &node : Base::nodes_) {
        visualization_msgs::msg::Marker labelMarker;
        labelMarker.header.frame_id = frameId;
        labelMarker.header.stamp    = timestamp;
        labelMarker.ns              = "node_labels";
        labelMarker.id              = labelId++;
        labelMarker.type    = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        labelMarker.action  = visualization_msgs::msg::Marker::ADD;
        labelMarker.scale.z = nodeScale * 2.0;  // Text size
        labelMarker.color.r = 1.0;
        labelMarker.color.g = 1.0;
        labelMarker.color.b = 1.0;
        labelMarker.color.a = 1.0;

        labelMarker.pose.position.x = node.pose_.x() + 0.1;
        labelMarker.pose.position.y = node.pose_.y() + 0.1;
        labelMarker.pose.position.z = nodeScale * 2.0;  // Above the node
        labelMarker.text            = std::to_string(node.id_);

        markerArray.markers.push_back(labelMarker);
      }
    }

    // Add link direction arrows if requested
    if (showLinkDirections) {
      int arrowId = 200;  // Start arrow IDs from 200
      for (const auto &[fromNode, neighbors] : Base::adj_list_) {
        const auto &fromPose = Base::get_pose(fromNode);

        for (const auto &[toNode, weight] : neighbors) {
          const auto &toPose = Base::get_pose(toNode);

          visualization_msgs::msg::Marker arrowMarker;
          arrowMarker.header.frame_id = frameId;
          arrowMarker.header.stamp    = timestamp;
          arrowMarker.ns              = "link_directions";
          arrowMarker.id              = arrowId++;
          arrowMarker.type            = visualization_msgs::msg::Marker::ARROW;
          arrowMarker.action          = visualization_msgs::msg::Marker::ADD;
          arrowMarker.scale.x         = edgeScale * 3.0;  // Arrow length
          arrowMarker.scale.y         = edgeScale * 1.0;  // Arrow width
          arrowMarker.scale.z         = edgeScale * 1.0;  // Arrow height

          arrowMarker.color.r = 0.0;
          arrowMarker.color.g = 0.0;
          arrowMarker.color.b = 1.0;
          arrowMarker.color.a = 0.8;

          // Calculate arrow position and orientation
          double dx     = toPose.x() - fromPose.x();
          double dy     = toPose.y() - fromPose.y();
          double length = std::sqrt(dx * dx + dy * dy);

          if (length > 1e-6) {
            // Position arrow at 70% along the link
            double t                    = 0.7;
            arrowMarker.pose.position.x = fromPose.x() + t * dx;
            arrowMarker.pose.position.y = fromPose.y() + t * dy;
            arrowMarker.pose.position.z = 0.0;

            // Orient arrow in link direction
            double          yaw = std::atan2(dy, dx);
            tf2::Quaternion q;
            q.setRPY(0, 0, yaw);
            arrowMarker.pose.orientation.x = q.x();
            arrowMarker.pose.orientation.y = q.y();
            arrowMarker.pose.orientation.z = q.z();
            arrowMarker.pose.orientation.w = q.w();

            markerArray.markers.push_back(arrowMarker);
          }
        }
      }
    }

    return markerArray;
  }
};

}  // namespace vrobot_route