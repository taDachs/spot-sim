#pragma once

#include <cstdint>
#include <random>
#include <vector>

#include "spot_sim_planning/grid.hpp"

namespace spot_sim::planning {

/// Planar pose in the map frame.
struct Pose2 {
  double x = 0.0, y = 0.0, yaw = 0.0;
};

/// Body velocity in the gravity-aligned body frame [m/s, m/s, rad/s].
struct Velocity2 {
  double vx = 0.0, vy = 0.0, wz = 0.0;
};

/// Pose after moving at body velocity `v` for `dt` from `p` (midpoint heading).
Pose2 Integrate(const Pose2& p, const Velocity2& v, double dt);

struct MppiParams {
  int samples = 1000;
  int horizon_steps = 40;  ///< with dt: a 2 s look-ahead
  double dt = 0.05;        ///< rollout step [s]; also the expected replanning period
  /// Relative to the cost spread of the best tenth of the samples: a sample
  /// that much worse than the best gets weight 1/e.
  double temperature = 0.2;
  /// Exploration noise per control axis, correlated over time:
  /// e_t = noise_correlation * e_{t-1} + sqrt(1 - c^2) * N(0, sigma).
  Velocity2 noise_sigma{0.5, 0.3, 0.5};
  double noise_correlation = 0.7;

  /// The robot as circles in the body frame (centres) of one radius: Spot's
  /// body is about 1.1 x 0.5 m.
  std::vector<Point2> footprint{{-0.35, 0.0}, {0.0, 0.0}, {0.35, 0.0}};
  double footprint_radius = 0.28;

  // Cost weights.
  double collision_cost = 1e4;     ///< per step with a footprint circle inside an obstacle
  double padding_weight = 200.0;   ///< (padding intrusion / padding)^2 per step
  double progress_weight = 2.0;    ///< cost-to-go per step [per m] (goal mode)
  double terminal_weight = 20.0;   ///< cost-to-go at the end of the horizon (goal mode)
  double heading_weight = 4.0;     ///< heading error^2 near the goal (goal mode)
  double arrive_weight = 2.0;      ///< speed^2 near the goal (goal mode)
  double goal_near_distance = 1.0; ///< within this, heading and speed at the goal count [m]
  double tracking_weight = 4.0;    ///< (u - commanded)^2 per step (velocity mode)
  /// Heading error^2 against the commanded heading (start yaw + commanded
  /// turn rate * t) per step (velocity mode): the robot slides along an
  /// obstacle instead of turning away on its own.
  double heading_tracking_weight = 40.0;
  double smoothness_weight = 0.5;  ///< (u_t - u_{t-1})^2 per step
  double unreachable_cost = 200.0;  ///< per step where the cost-to-go is not finite

  uint32_t seed = 1;
};

/// What to plan for in one cycle.
struct MppiProblem {
  enum class Mode { kGoal, kVelocity };
  Mode mode = Mode::kVelocity;

  Pose2 start;
  Velocity2 current;  ///< the body velocity now (the first control's acceleration starts from it)

  /// Goal mode: the goal pose and the cost-to-go field toward it, extended
  /// into the inflated cells next to obstacles (ExtendCostToGo()) so that a
  /// robot standing close to a wall still knows the way. Outside the field's
  /// window the straight-line distance to the goal stands in.
  Pose2 goal;
  const Grid<float>* cost_to_go = nullptr;
  /// Velocity mode: the commanded body velocity.
  Velocity2 commanded;

  /// Signed distance to obstacles (SignedDistance()).
  const Grid<float>* distance = nullptr;
  /// Extra clearance the robot keeps from obstacles [m] (obstacle_avoidance_padding).
  double padding = 0.1;

  /// Velocity and acceleration limits (from the command).
  Velocity2 max_velocity{1.6, 0.6, 1.2};
  Velocity2 max_accel{2.0, 2.0, 2.0};
};

struct MppiResult {
  Velocity2 command;                ///< apply until the next cycle
  std::vector<Velocity2> controls;  ///< the optimized control sequence
  std::vector<Pose2> trajectory;    ///< poses under `controls` (horizon_steps + 1)
  bool collision_free = false;      ///< the optimized trajectory keeps clear of obstacles
  double cost = 0.0;                ///< of the best sample
};

/// Model predictive path integral control (Williams et al.) for a holonomic
/// base: samples noisy velocity sequences around the previous solution,
/// rolls them out, and averages them weighted by exp(-cost / temperature).
///
/// Obstacles come from the distance field (the 2.5D map, moving things
/// included: they are just redrawn every cycle). Dead ends are avoided by
/// the cost-to-go field, which knows the way around.
///
/// Deterministic: the same sequence of problems gives the same results.
class Mppi {
 public:
  explicit Mppi(MppiParams params = {});

  const MppiParams& params() const { return params_; }

  /// One planning cycle. Shifts the previous solution by one step (call it
  /// every `dt`) and optimizes it.
  MppiResult Plan(const MppiProblem& problem);

  /// Forgets the previous solution (e.g. when the goal changes a lot).
  void Reset();

 private:
  /// Rolls out `controls` from the problem's start; returns the cost and
  /// whether a footprint circle hit an obstacle.
  double Rollout(const MppiProblem& problem, const std::vector<Velocity2>& controls, std::vector<Pose2>* poses,
                 bool* collided) const;

  MppiParams params_;
  std::vector<Velocity2> nominal_;
  std::mt19937 rng_;
};

}  // namespace spot_sim::planning
