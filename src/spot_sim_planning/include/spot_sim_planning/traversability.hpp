#pragma once

#include <cstdint>

#include "spot_sim_planning/grid.hpp"

namespace spot_sim::planning {

/// Where Spot can put its feet. The limits match what the locomotion policy
/// is trained for (docs/handoff/rl_locomotion.md: steps up to 15 cm, stairs
/// with 17 cm risers, slopes up to 20 deg).
struct TraversabilityParams {
  /// Largest height difference within a 3x3 neighbourhood that is still a
  /// step the robot can take [m].
  double max_step = 0.18;
  /// Steepest surface the robot walks on [rad].
  double max_slope = 25.0 * 3.14159265358979323846 / 180.0;
  /// Radius of the plane fit for slopes [m] (a (2k+1)^2 cell window).
  double slope_radius = 0.1;
  /// A window fits a plane (so its slope counts) only if the RMS residual is
  /// below this [m]; otherwise it's a step edge or rough, and the step rule
  /// decides. Keeps stair edges from reading as steep slopes.
  double max_plane_residual = 0.02;
  /// Unknown cells are obstacles (true) or free (false).
  bool unknown_is_obstacle = true;
  /// Extra cost per unit of slope (fraction of max_slope) and of step height
  /// (fraction of max_step), on top of 1 for flat ground.
  double slope_cost = 1.0;
  double step_cost = 1.0;
};

struct Traversability {
  /// 1 where the robot cannot walk (too high a step, too steep, unknown).
  Grid<uint8_t> obstacle;
  /// Relative cost of walking through a cell: 1 on flat ground, more on slopes
  /// and steps; meaningless on obstacles.
  Grid<float> cost;
  /// Local slope [rad] (0 where no plane fits) and step height [m], for
  /// debugging and tests.
  Grid<float> slope;
  Grid<float> step;
};

/// Classifies every cell of `heights` (NaN = unknown).
Traversability ComputeTraversability(const Grid<float>& heights, const TraversabilityParams& params = {});

}  // namespace spot_sim::planning
