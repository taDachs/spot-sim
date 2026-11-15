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

/// `cost_to_go` with its unreachable cells within `radius` [m] of a reachable
/// one filled in: the reachable value plus `weight` times the distance. Lets
/// a planner standing in the inflation zone next to an obstacle (impassable
/// for paths, but where the robot can be) still find the way out.
Grid<float> ExtendCostToGo(const Grid<float>& cost_to_go, double radius, double weight);

}  // namespace spot_sim::planning
