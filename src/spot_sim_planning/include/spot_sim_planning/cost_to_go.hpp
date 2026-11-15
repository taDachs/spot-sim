#pragma once

#include "spot_sim_planning/grid.hpp"
#include "spot_sim_planning/traversability.hpp"

namespace spot_sim::planning {

struct CostToGoParams {
  /// Cells closer than this to an obstacle are impassable [m]: the robot's
  /// half-width plus the obstacle padding.
  double inflation_radius = 0.35;
  /// Within this distance of an obstacle the cost rises toward
  /// `proximity_weight` extra (linearly), so paths keep some room [m].
  double proximity_radius = 0.6;
  double proximity_weight = 2.0;
};

/// Cost [m of flat ground] of the cheapest 8-connected path from each cell to
/// the goal, over traversable cells (Dijkstra). Unreachable cells are +inf.
///
/// - Goal inside the window: the search starts at the goal's cell. If that
///   cell is impassable, everything is unreachable.
/// - Goal outside the window: the search starts at every passable border
///   cell, with the straight-line distance from the cell to the goal as its
///   initial cost. The field then guides toward the border cells on the way.
///
/// `distance` is SignedDistance(traversability.obstacle).
Grid<float> ComputeCostToGo(const Traversability& traversability, const Grid<float>& distance, Point2 goal,
                            const CostToGoParams& params = {});

}  // namespace spot_sim::planning
