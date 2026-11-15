#include "spot_sim_planning/mppi.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace spot_sim::planning {

namespace {

constexpr double kPi = 3.14159265358979323846;

double WrapAngle(double a) { return std::remainder(a, 2.0 * kPi); }

/// Nearest-cell lookup; `outside` beyond the grid.
double Lookup(const Grid<float>& field, Point2 p, double outside) {
  const auto c = field.geometry().CellAt(p);
  return c ? static_cast<double>(field.at(*c)) : outside;
}

double Clamp(double v, double limit) { return std::clamp(v, -limit, limit); }

/// `u` within the velocity limits and reachable from `previous` within one
/// step of the acceleration limits.
Velocity2 Limit(Velocity2 u, const Velocity2& previous, const MppiProblem& problem, double dt) {
  u.vx = Clamp(u.vx, problem.max_velocity.vx);
  u.vy = Clamp(u.vy, problem.max_velocity.vy);
  u.wz = Clamp(u.wz, problem.max_velocity.wz);
  u.vx = previous.vx + Clamp(u.vx - previous.vx, problem.max_accel.vx * dt);
  u.vy = previous.vy + Clamp(u.vy - previous.vy, problem.max_accel.vy * dt);
  u.wz = previous.wz + Clamp(u.wz - previous.wz, problem.max_accel.wz * dt);
  return u;
}

}  // namespace

Pose2 Integrate(const Pose2& p, const Velocity2& v, double dt) {
  const double heading = p.yaw + 0.5 * v.wz * dt;
  const double c = std::cos(heading), s = std::sin(heading);
  return Pose2{p.x + (c * v.vx - s * v.vy) * dt, p.y + (s * v.vx + c * v.vy) * dt, WrapAngle(p.yaw + v.wz * dt)};
}

Mppi::Mppi(MppiParams params) : params_(std::move(params)), rng_(params_.seed) {}

void Mppi::Reset() { nominal_.clear(); }

double Mppi::Rollout(const MppiProblem& problem, const std::vector<Velocity2>& controls, std::vector<Pose2>* poses,
                     bool* collided) const {
  const MppiParams& p = params_;
  const double half_cell = problem.distance->geometry().resolution() / 2.0;
  const bool goal_mode = problem.mode == MppiProblem::Mode::kGoal;
  const auto cost_to_go = [&](const Pose2& pose) {
    const Point2 at{pose.x, pose.y};
    return Lookup(*problem.cost_to_go, at, (at - Point2{problem.goal.x, problem.goal.y}).Norm());
  };

  double cost = 0.0;
  Pose2 pose = problem.start;
  Velocity2 previous = problem.current;
  *collided = false;
  if (poses) poses->assign(1, pose);
  for (std::size_t step = 0; step < controls.size(); ++step) {
    const Velocity2& u = controls[step];
    cost += p.smoothness_weight * ((u.vx - previous.vx) * (u.vx - previous.vx) +
                                   (u.vy - previous.vy) * (u.vy - previous.vy) +
                                   (u.wz - previous.wz) * (u.wz - previous.wz));
    previous = u;
    pose = Integrate(pose, u, p.dt);
    if (poses) poses->push_back(pose);

    const double c = std::cos(pose.yaw), s = std::sin(pose.yaw);
    for (const Point2& circle : p.footprint) {
      const Point2 at{pose.x + c * circle.x - s * circle.y, pose.y + s * circle.x + c * circle.y};
      const double clearance =
          Lookup(*problem.distance, at, std::numeric_limits<double>::infinity()) - p.footprint_radius - half_cell;
      if (clearance < 0.0) {
        cost += p.collision_cost;
        *collided = true;
      } else if (clearance < problem.padding) {
        const double intrusion = (problem.padding - clearance) / problem.padding;
        cost += p.padding_weight * intrusion * intrusion;
      }
    }

    if (goal_mode) {
      const double to_go = cost_to_go(pose);
      cost += std::isfinite(to_go) ? p.progress_weight * to_go : p.unreachable_cost;
      const double distance = std::hypot(pose.x - problem.goal.x, pose.y - problem.goal.y);
      if (distance < p.goal_near_distance) {
        const double nearness = 1.0 - distance / p.goal_near_distance;
        const double heading_error = WrapAngle(pose.yaw - problem.goal.yaw);
        cost += nearness * (p.heading_weight * heading_error * heading_error +
                            p.arrive_weight * (u.vx * u.vx + u.vy * u.vy + 0.25 * u.wz * u.wz));
      }
    } else {
      const Velocity2& v = problem.commanded;
      cost += p.tracking_weight *
              ((u.vx - v.vx) * (u.vx - v.vx) + (u.vy - v.vy) * (u.vy - v.vy) + (u.wz - v.wz) * (u.wz - v.wz));
      const double heading_error = WrapAngle(pose.yaw - (problem.start.yaw + v.wz * (step + 1) * p.dt));
      cost += p.heading_tracking_weight * heading_error * heading_error;
    }
  }
  if (goal_mode) {
    const double to_go = cost_to_go(pose);
    cost += std::isfinite(to_go) ? p.terminal_weight * to_go : p.unreachable_cost * p.horizon_steps;
  }
  return cost;
}

double Mppi::MinClearance(const MppiProblem& problem, const std::vector<Velocity2>& controls,
                          std::vector<Pose2>* poses) const {
  const MppiParams& p = params_;
  const double half_cell = problem.distance->geometry().resolution() / 2.0;
  double clearance = std::numeric_limits<double>::infinity();
  Pose2 pose = problem.start;
  if (poses) poses->assign(1, pose);
  for (const Velocity2& u : controls) {
    pose = Integrate(pose, u, p.dt);
    if (poses) poses->push_back(pose);
    const double c = std::cos(pose.yaw), s = std::sin(pose.yaw);
    for (const Point2& circle : p.footprint) {
      const Point2 at{pose.x + c * circle.x - s * circle.y, pose.y + s * circle.x + c * circle.y};
      clearance = std::min(clearance, Lookup(*problem.distance, at, std::numeric_limits<double>::infinity()) -
                                          p.footprint_radius - half_cell);
    }
  }
  return clearance;
}

MppiResult Mppi::Plan(const MppiProblem& problem) {
  const MppiParams& p = params_;
  const int horizon = p.horizon_steps;

  // Warm start: the previous solution, one step on.
  if (static_cast<int>(nominal_.size()) != horizon) {
    nominal_.assign(horizon, problem.current);
  } else {
    std::rotate(nominal_.begin(), nominal_.begin() + 1, nominal_.end());
    nominal_.back() = nominal_[horizon - 2];
  }

  // Velocity mode: where doing exactly what was asked keeps clear of
  // obstacles (with the padding) over the whole horizon, do just that, as the
  // robot does in free space. MPPI only steps in near obstacles, starting
  // from the command.
  if (problem.mode == MppiProblem::Mode::kVelocity) {
    std::vector<Velocity2> commanded(horizon);
    Velocity2 previous = problem.current;
    for (int t = 0; t < horizon; ++t) {
      commanded[t] = Limit(problem.commanded, previous, problem, p.dt);
      previous = commanded[t];
    }
    const int look_ahead = std::clamp(static_cast<int>(std::lround(p.passthrough_horizon / p.dt)), 1, horizon);
    const std::vector<Velocity2> near_term(commanded.begin(), commanded.begin() + look_ahead);
    MppiResult result;
    if (MinClearance(problem, near_term, nullptr) >= problem.padding) {
      MinClearance(problem, commanded, &result.trajectory);  // for visualization
      nominal_ = commanded;
      result.controls = commanded;
      result.command = problem.commanded;  // the base ramps up to it itself
      result.collision_free = true;
      return result;
    }
  }

  std::normal_distribution<double> normal(0.0, 1.0);
  const double fresh = std::sqrt(1.0 - p.noise_correlation * p.noise_correlation);
  std::vector<std::vector<Velocity2>> samples(p.samples, std::vector<Velocity2>(horizon));
  std::vector<double> costs(p.samples);
  bool collided = false;
  for (int k = 0; k < p.samples; ++k) {
    std::vector<Velocity2>& u = samples[k];
    Velocity2 noise, previous = problem.current;
    for (int t = 0; t < horizon; ++t) {
      Velocity2 target = nominal_[t];
      if (k == 1) {
        target = Velocity2{};  // always consider stopping
      } else if (k == 2 && problem.mode == MppiProblem::Mode::kVelocity) {
        target = problem.commanded;  // and doing exactly what was asked
      } else if (k > 1) {
        noise.vx = p.noise_correlation * noise.vx + fresh * p.noise_sigma.vx * normal(rng_);
        noise.vy = p.noise_correlation * noise.vy + fresh * p.noise_sigma.vy * normal(rng_);
        noise.wz = p.noise_correlation * noise.wz + fresh * p.noise_sigma.wz * normal(rng_);
        target = Velocity2{target.vx + noise.vx, target.vy + noise.vy, target.wz + noise.wz};
      }  // k == 0: the previous solution as is
      u[t] = Limit(target, previous, problem, p.dt);
      previous = u[t];
    }
    costs[k] = Rollout(problem, u, nullptr, &collided);
  }

  // Exponentially weighted average. The temperature is relative to the
  // spread of the better samples (best to 10th percentile), so it doesn't
  // depend on the costs' scale and colliding samples can't flatten it.
  const double best = *std::min_element(costs.begin(), costs.end());
  std::vector<double> sorted = costs;
  const auto decile = sorted.begin() + p.samples / 10;
  std::nth_element(sorted.begin(), decile, sorted.end());
  const double lambda = std::max(1e-9, p.temperature * (*decile - best));
  std::vector<double> weights(p.samples);
  double total = 0.0;
  for (int k = 0; k < p.samples; ++k) {
    weights[k] = std::exp(-(costs[k] - best) / lambda);
    total += weights[k];
  }
  for (int t = 0; t < horizon; ++t) {
    Velocity2 sum;
    for (int k = 0; k < p.samples; ++k) {
      const double w = weights[k] / total;
      sum.vx += w * samples[k][t].vx;
      sum.vy += w * samples[k][t].vy;
      sum.wz += w * samples[k][t].wz;
    }
    nominal_[t] = sum;
  }

  MppiResult result;
  result.controls = nominal_;
  result.command = nominal_.front();
  result.cost = best;
  bool nominal_collided = false;
  Rollout(problem, nominal_, &result.trajectory, &nominal_collided);
  result.collision_free = !nominal_collided;
  return result;
}

}  // namespace spot_sim::planning
