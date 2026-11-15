#include "spot_sim_planning/local_planner.hpp"

#include <cmath>

#include "spot_sim_planning/distance_field.hpp"

namespace spot_sim::planning {

namespace {
double WrapAngle(double a) { return std::remainder(a, 2.0 * 3.14159265358979323846); }
}  // namespace

LocalPlanner::LocalPlanner(LocalPlannerConfig config)
    : config_(std::move(config)), map_(config_.map_size, config_.map_resolution), mppi_(config_.mppi) {}

void LocalPlanner::UpdateMap(const LocalPlannerInput& input) {
  const double ground_z = input.robot_z - config_.stand_height;
  const double z_top = ground_z + config_.obstacle_ceiling;
  const bool moved = map_.RecenterIfNeeded({input.robot.x, input.robot.y}, config_.recenter_slack);
  if (input.static_version != static_version_ ||
      (static_ground_z_ && std::abs(ground_z - *static_ground_z_) > config_.ceiling_refresh)) {
    map_.InvalidateStatic();
    static_version_ = input.static_version;
    static_ground_z_.reset();
  }
  if (input.static_shapes && !map_.UnknownStaticCells().empty()) {
    FillStatic(map_, *input.static_shapes, z_top);
    if (!static_ground_z_) static_ground_z_ = ground_z;
  }
  DrawMovable(map_, input.movable_shapes, z_top);
  traversability_ = ComputeTraversability(map_.Heights(), config_.traversability);
  distance_ = SignedDistance(traversability_.obstacle);
  if (moved) cost_to_go_valid_ = false;  // its grid no longer matches
}

LocalPlannerOutput LocalPlanner::Step(const LocalPlannerInput& input) {
  UpdateMap(input);
  if (last_mode_ != input.mode) {
    mppi_.Reset();
    goal_sequence_.reset();
    last_mode_ = input.mode;
  }
  if (input.mode == LocalPlannerInput::Mode::kGoal) {
    return StepGoal(input);
  }
  MppiProblem problem;
  problem.mode = MppiProblem::Mode::kVelocity;
  problem.start = input.robot;
  problem.current = input.current;
  problem.commanded = input.commanded;
  problem.distance = &distance_;
  problem.padding = input.padding;
  problem.max_velocity = input.max_velocity;
  problem.max_accel = input.max_accel;
  LocalPlannerOutput out;
  out.mppi = mppi_.Plan(problem);
  out.command = out.mppi.command;
  out.state = LocalPlannerState::kMoving;
  return out;
}

LocalPlannerOutput LocalPlanner::StepGoal(const LocalPlannerInput& input) {
  LocalPlannerOutput out;
  if (goal_sequence_ != input.goal_sequence) {
    goal_sequence_ = input.goal_sequence;
    mppi_.Reset();
    at_goal_ = false;
    cost_to_go_valid_ = false;
    best_cost_to_go_ = INFINITY;
    last_progress_time_ = input.time;
  }

  const double distance = std::hypot(input.goal.x - input.robot.x, input.goal.y - input.robot.y);
  const double yaw_error = std::abs(WrapAngle(input.goal.yaw - input.robot.yaw));
  if (at_goal_ && (distance > 2.0 * input.position_tolerance || yaw_error > 2.0 * input.yaw_tolerance)) {
    at_goal_ = false;  // pushed off the goal: go back
    mppi_.Reset();
    last_progress_time_ = input.time;
  } else if (!at_goal_ && distance <= input.position_tolerance && yaw_error <= input.yaw_tolerance) {
    at_goal_ = true;
  }
  if (at_goal_) {
    mppi_.Reset();
    out.state = LocalPlannerState::kAtGoal;
    return out;  // zero command
  }

  if (!input.avoid) {
    // Straight for the goal: no obstacles, cost-to-go = distance.
    const GridGeometry& g = distance_.geometry();
    Grid<float> free(g, INFINITY), straight(g, 0.0f);
    for (std::size_t i = 0; i < straight.size(); ++i) {
      straight[i] = static_cast<float>((g.Center(g.FromIndex(i)) - Point2{input.goal.x, input.goal.y}).Norm());
    }
    MppiProblem problem;
    problem.mode = MppiProblem::Mode::kGoal;
    problem.start = input.robot;
    problem.current = input.current;
    problem.goal = input.goal;
    problem.cost_to_go = &straight;
    problem.distance = &free;
    problem.padding = input.padding;
    problem.max_velocity = input.max_velocity;
    problem.max_accel = input.max_accel;
    out.mppi = mppi_.Plan(problem);
    out.command = out.mppi.command;
    out.state = distance < config_.near_goal_distance ? LocalPlannerState::kNearGoal : LocalPlannerState::kMoving;
    cost_to_go_valid_ = false;
    return out;
  }

  if (!cost_to_go_valid_ || ++cycles_since_cost_to_go_ >= config_.cost_to_go_every) {
    cost_to_go_ = ExtendCostToGo(
        ComputeCostToGo(traversability_, distance_, {input.goal.x, input.goal.y}, config_.cost_to_go),
        config_.cost_to_go_extension, config_.cost_to_go_off_weight);
    cost_to_go_valid_ = true;
    cycles_since_cost_to_go_ = 0;
  }

  MppiProblem problem;
  problem.mode = MppiProblem::Mode::kGoal;
  problem.start = input.robot;
  problem.current = input.current;
  problem.goal = input.goal;
  problem.cost_to_go = &cost_to_go_;
  problem.distance = &distance_;
  problem.padding = input.padding;
  problem.max_velocity = input.max_velocity;
  problem.max_accel = input.max_accel;
  out.mppi = mppi_.Plan(problem);
  out.command = out.mppi.command;

  // Progress: the cost-to-go at the robot keeps going down.
  const auto cell = cost_to_go_.geometry().CellAt({input.robot.x, input.robot.y});
  const double to_go = cell ? cost_to_go_.at(*cell) : distance;
  if (std::isfinite(to_go) && to_go < best_cost_to_go_ - config_.progress_epsilon) {
    best_cost_to_go_ = to_go;
    last_progress_time_ = input.time;
  }
  // Turning toward the way out, or backing up to get around something, is
  // no progress on the cost-to-go but not blocked either: only a robot that
  // the planner keeps (nearly) still counts.
  const bool moving = std::hypot(out.command.vx, out.command.vy) > config_.stall_speed ||
                      std::abs(out.command.wz) > config_.stall_turn_rate;
  if (moving) last_progress_time_ = input.time;
  const bool reachable = std::isfinite(to_go);
  // Near the goal the robot may only be turning, which doesn't lower the
  // cost-to-go: no time-out there.
  const bool near = distance < config_.near_goal_distance;
  if (near) last_progress_time_ = input.time;
  if (!reachable || input.time - last_progress_time_ > config_.blocked_after) {
    out.state = LocalPlannerState::kBlocked;
    // Keep trying, and count progress from here on: if the way clears, the
    // robot moves again and the state goes back to moving.
    if (reachable) best_cost_to_go_ = to_go;
  } else {
    out.state = near ? LocalPlannerState::kNearGoal : LocalPlannerState::kMoving;
  }
  return out;
}

}  // namespace spot_sim::planning
