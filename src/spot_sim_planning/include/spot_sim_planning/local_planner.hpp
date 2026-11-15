#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include "spot_sim_planning/cost_to_go.hpp"
#include "spot_sim_planning/height_map.hpp"
#include "spot_sim_planning/mppi.hpp"
#include "spot_sim_planning/traversability.hpp"
#include "spot_sim_planning/world_shapes.hpp"

namespace spot_sim::planning {

struct LocalPlannerConfig {
  int map_size = 160;            ///< cells per side (8 m at 5 cm)
  double map_resolution = 0.05;  ///< [m]
  double recenter_slack = 1.0;   ///< the window follows the robot in steps of this [m]
  /// Things higher than this above the robot's feet are ignored (the robot
  /// walks under them) [m].
  double obstacle_ceiling = 0.8;
  /// Robot body origin above its feet when standing [m]; gives the feet's
  /// height from the body pose.
  double stand_height = 0.52;
  /// The static layer is redrawn when the robot's feet change height by more
  /// than this (the ceiling moved with them) [m].
  double ceiling_refresh = 0.3;

  TraversabilityParams traversability;
  CostToGoParams cost_to_go;
  double cost_to_go_extension = 0.6;  ///< ExtendCostToGo radius [m]
  double cost_to_go_off_weight = 20.0;
  int cost_to_go_every = 4;  ///< cycles between cost-to-go updates (5 Hz at 20 Hz)

  MppiParams mppi;

  double near_goal_distance = 0.5;  ///< [m]
  /// Blocked: the goal is unreachable in the cost-to-go field, or no
  /// progress (cost-to-go down by at least `progress_epsilon`) for
  /// `blocked_after` [s] while farther than near_goal_distance from it ...
  double blocked_after = 1.5;
  double progress_epsilon = 0.05;
  /// ... and the planner's command stays below these (it keeps the robot
  /// nearly still) [m/s, rad/s].
  double stall_speed = 0.1;
  double stall_turn_rate = 0.15;
};

/// What one planning cycle sees. Poses are in the map frame (the simulator's
/// world frame).
struct LocalPlannerInput {
  double time = 0.0;  ///< [s], monotonic (simulation time)
  Pose2 robot;
  double robot_z = 0.0;  ///< body origin height [m]
  Velocity2 current;     ///< body velocity now (the last command)

  enum class Mode { kVelocity, kGoal };
  Mode mode = Mode::kVelocity;
  Velocity2 commanded;  ///< kVelocity
  Pose2 goal;           ///< kGoal
  uint32_t goal_sequence = 0;  ///< a new value means a new goal
  double position_tolerance = 0.1;  ///< [m]
  double yaw_tolerance = 0.1;       ///< [rad]

  /// Avoid obstacles (kGoal; in kVelocity the caller skips the planner
  /// instead). Without it the robot heads straight for the goal, through
  /// whatever is in the way, as the robot does with vision obstacle
  /// avoidance disabled.
  bool avoid = true;
  double padding = 0.1;  ///< obstacle_avoidance_padding [m]
  Velocity2 max_velocity{1.6, 0.6, 1.2};
  Velocity2 max_accel{2.0, 2.0, 2.0};

  /// The world: static shapes (shared; replaced as a whole with a new
  /// `static_version` when they change) and movable ones (every cycle).
  std::shared_ptr<const std::vector<Shape>> static_shapes;
  uint64_t static_version = 0;
  std::vector<Shape> movable_shapes;
};

enum class LocalPlannerState {
  kMoving,    ///< following the command or on the way to the goal
  kNearGoal,  ///< within near_goal_distance, still settling
  kAtGoal,    ///< within the tolerances; standing still
  kBlocked,   ///< no way to the goal right now (keeps trying)
};

struct LocalPlannerOutput {
  Velocity2 command;
  LocalPlannerState state = LocalPlannerState::kMoving;
  MppiResult mppi;  ///< for visualization (empty when at the goal)
};

/// The local planner's cycle (call it every MppiParams::dt of simulation
/// time): updates the robot-centred map from the world's shapes, derives
/// traversability, the distance field and the cost-to-go, and runs MPPI.
/// In goal mode it also decides when the goal is reached or blocked.
class LocalPlanner {
 public:
  explicit LocalPlanner(LocalPlannerConfig config = {});

  const LocalPlannerConfig& config() const { return config_; }
  LocalPlannerOutput Step(const LocalPlannerInput& input);

  const HeightMap& map() const { return map_; }
  const Traversability& traversability() const { return traversability_; }
  const Grid<float>& distance() const { return distance_; }

 private:
  void UpdateMap(const LocalPlannerInput& input);
  LocalPlannerOutput StepGoal(const LocalPlannerInput& input);

  LocalPlannerConfig config_;
  HeightMap map_;
  Mppi mppi_;
  Traversability traversability_;
  Grid<float> distance_;
  Grid<float> cost_to_go_;

  uint64_t static_version_ = 0;
  std::optional<double> static_ground_z_;  ///< feet height the static layer was drawn for
  int cycles_since_cost_to_go_ = 0;
  bool cost_to_go_valid_ = false;
  std::optional<LocalPlannerInput::Mode> last_mode_;

  // Goal mode.
  std::optional<uint32_t> goal_sequence_;
  bool at_goal_ = false;
  double best_cost_to_go_ = 0.0;
  double last_progress_time_ = 0.0;
};

}  // namespace spot_sim::planning
