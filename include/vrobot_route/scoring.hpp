#pragma once

#include "vrobot_route/datastructor.hpp"
#include <vrobot_route/pathfinding.hpp>

namespace vrobot_route {

class ScoringSystem : public virtual VGraph {
public:
  using Base = VGraph;

  // ========================================================================
  // WEIGHTED SCORING FUNCTIONS
  // ========================================================================

  /**
   * @brief Calculate weighted score for link-based path planning
   * @param linkDistance Distance from pose to link
   * @param graphDistance Distance through graph from link to target
   * @param linkWeight Weight for link distance component
   * @param graphWeight Weight for graph distance component
   * @return Weighted score (lower is better)
   */
  inline double calculate_weighted_score(double linkDistance,
                                         double graphDistance,
                                         double linkWeight  = 2.0,
                                         double graphWeight = 1.0) const {
    return (linkDistance * linkWeight) + (graphDistance * graphWeight);
  }

  /**
   * @brief Calculate normalized weighted score
   * @param linkDistance Distance from pose to link
   * @param graphDistance Distance through graph from link to target
   * @param linkWeight Weight for link distance component
   * @param graphWeight Weight for graph distance component
   * @param maxLinkDistance Maximum link distance for normalization
   * @param maxGraphDistance Maximum graph distance for normalization
   * @return Normalized weighted score (0-1 range, lower is better)
   */
  inline double calculate_normalized_weighted_score(
      double linkDistance, double graphDistance, double linkWeight = 2.0,
      double graphWeight = 1.0, double maxLinkDistance = 1.0,
      double maxGraphDistance = 10.0) const {
    double normalizedLinkDist = std::min(linkDistance / maxLinkDistance, 1.0);
    double normalizedGraphDist =
        std::min(graphDistance / maxGraphDistance, 1.0);

    double totalWeight = linkWeight + graphWeight;
    return ((normalizedLinkDist * linkWeight) +
            (normalizedGraphDist * graphWeight)) /
           totalWeight;
  }
};

} // namespace vrobot_route