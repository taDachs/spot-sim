#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <functional>

#include "spot_sim_planning/cost_to_go.hpp"
#include "spot_sim_planning/distance_field.hpp"
#include "spot_sim_planning/mppi.hpp"
#include "spot_sim_planning/traversability.hpp"

using spot_sim::planning::ComputeCostToGo;
using spot_sim::planning::ComputeTraversability;
using spot_sim::planning::ExtendCostToGo;
using spot_sim::planning::Grid;
using spot_sim::planning::GridGeometry;
using spot_sim::planning::Integrate;
using spot_sim::planning::Mppi;
using spot_sim::planning::MppiProblem;
using spot_sim::planning::MppiResult;
using spot_sim::planning::Point2;
using spot_sim::planning::Pose2;
using spot_sim::planning::SignedDistance;
using spot_sim::planning::Velocity2;

namespace {

constexpr double kPi = 3.14159265358979323846;

/// Obstacles (1 m high) as a function of place and time.
using World = std::function<bool(Point2, double)>;

struct Layers {
  Grid<float> distance;
  Grid<float> cost_to_go;
};

/// The map pipeline the planner plugin runs: heights -> traversability ->
/// distance -> cost-to-go, on an 8 x 8 m window at 5 cm centred on `center`.
Layers Build(const World& world, double t, Point2 center, Point2 goal) {
  Grid<float> heights(GridGeometry(0.05, 160, 160, Point2{center.x - 4.0, center.y - 4.0}), 0.0f);
  for (std::size_t i = 0; i < heights.size(); ++i) {
    heights[i] = world(heights.geometry().Center(heights.geometry().FromIndex(i)), t) ? 1.0f : 0.0f;
  }
  const auto traversability = ComputeTraversability(heights);
  Layers layers;
  layers.distance = SignedDistance(traversability.obstacle);
  layers.cost_to_go = ExtendCostToGo(ComputeCostToGo(traversability, layers.distance, goal), 0.5, 20.0);
  return layers;
}

/// Smallest clearance of the footprint circles at `pose` (negative: collision).
double Clearance(const Mppi& mppi, const Grid<float>& distance, const Pose2& pose) {
  double clearance = INFINITY;
  const double c = std::cos(pose.yaw), s = std::sin(pose.yaw);
  for (const Point2& circle : mppi.params().footprint) {
    const Point2 at{pose.x + c * circle.x - s * circle.y, pose.y + s * circle.x + c * circle.y};
    const auto cell = distance.geometry().CellAt(at);
    if (!cell) continue;
    clearance = std::min(clearance, distance.at(*cell) - mppi.params().footprint_radius -
                                        distance.geometry().resolution() / 2.0);
  }
  return clearance;
}

struct Outcome {
  Pose2 pose;
  Velocity2 velocity;
  double min_clearance = INFINITY;
  int steps = 0;
  bool arrived = false;
};

/// Closed loop: plan, apply the command for one step, repeat. The robot
/// follows the commanded velocity exactly. Stops early once within the goal
/// tolerance and slow (goal mode).
Outcome Simulate(Mppi& mppi, MppiProblem problem, const World& world, int max_steps, bool dynamic = false) {
  Outcome run;
  run.pose = problem.start;
  const Point2 goal{problem.goal.x, problem.goal.y};
  Layers layers = Build(world, 0.0, {run.pose.x, run.pose.y}, goal);
  for (int step = 0; step < max_steps; ++step) {
    const double t = step * mppi.params().dt;
    if (dynamic) layers = Build(world, t, {run.pose.x, run.pose.y}, goal);
    problem.start = run.pose;
    problem.current = run.velocity;
    problem.distance = &layers.distance;
    problem.cost_to_go = &layers.cost_to_go;
    const MppiResult result = mppi.Plan(problem);
    run.velocity = result.command;
    run.pose = Integrate(run.pose, run.velocity, mppi.params().dt);
    run.steps = step + 1;
    // Clearance against the world as it is now.
    const Layers now = dynamic ? Build(world, t + mppi.params().dt, {run.pose.x, run.pose.y}, goal) : layers;
    run.min_clearance = std::min(run.min_clearance, Clearance(mppi, now.distance, run.pose));
    if (problem.mode == MppiProblem::Mode::kGoal) {
      const double error = std::hypot(run.pose.x - goal.x, run.pose.y - goal.y);
      const double speed = std::hypot(run.velocity.vx, run.velocity.vy);
      if (error < 0.1 && std::abs(std::remainder(run.pose.yaw - problem.goal.yaw, 2 * kPi)) < 0.15 && speed < 0.2) {
        run.arrived = true;
        break;
      }
    }
  }
  return run;
}

MppiProblem GoalProblem(Pose2 start, Pose2 goal) {
  MppiProblem problem;
  problem.mode = MppiProblem::Mode::kGoal;
  problem.start = start;
  problem.goal = goal;
  return problem;
}

MppiProblem VelocityProblem(Pose2 start, Velocity2 commanded) {
  MppiProblem problem;
  problem.mode = MppiProblem::Mode::kVelocity;
  problem.start = start;
  problem.commanded = commanded;
  return problem;
}

bool Box(Point2 p, Point2 center, double half_x, double half_y) {
  return std::abs(p.x - center.x) < half_x && std::abs(p.y - center.y) < half_y;
}

}  // namespace

TEST(Mppi, ReachesAGoalInFreeSpace) {
  Mppi mppi;
  const Outcome run = Simulate(mppi, GoalProblem({-2.0, 0.0, 0.0}, {2.0, 1.0, kPi / 2}), [](Point2, double) { return false; },
                           400);
  EXPECT_TRUE(run.arrived) << run.pose.x << ", " << run.pose.y << ", " << run.pose.yaw;
  EXPECT_LT(run.steps, 300);  // about 4.5 m: well under 15 s
}

TEST(Mppi, GoesAroundABox) {
  Mppi mppi;
  const World world = [](Point2 p, double) { return Box(p, {0.0, 0.0}, 0.4, 0.4); };
  const Outcome run = Simulate(mppi, GoalProblem({-2.5, 0.0, 0.0}, {2.5, 0.0, 0.0}), world, 500);
  EXPECT_TRUE(run.arrived) << run.pose.x << ", " << run.pose.y;
  EXPECT_GT(run.min_clearance, 0.0);
}

TEST(Mppi, FindsTheGapInAWall) {
  Mppi mppi;
  // Wall across x = 0, open for y in [1.5, 2.7].
  const World world = [](Point2 p, double) { return std::abs(p.x) < 0.1 && !(p.y > 1.5 && p.y < 2.7); };
  const Outcome run = Simulate(mppi, GoalProblem({-2.0, 0.0, 0.0}, {2.0, 0.0, 0.0}), world, 800);
  EXPECT_TRUE(run.arrived) << run.pose.x << ", " << run.pose.y;
  EXPECT_GT(run.min_clearance, 0.0);
}

TEST(Mppi, NeverWalksIntoAClosedWall) {
  Mppi mppi;
  const World world = [](Point2 p, double) { return std::abs(p.x) < 0.1; };
  const Outcome run = Simulate(mppi, GoalProblem({-1.5, 0.0, 0.0}, {2.0, 0.0, 0.0}), world, 200);
  EXPECT_FALSE(run.arrived);
  EXPECT_GT(run.min_clearance, 0.0);
  EXPECT_LT(run.pose.x, 0.0);
}

TEST(Mppi, VelocityModeTracksTheCommandInFreeSpace) {
  Mppi mppi;
  const Outcome run = Simulate(mppi, VelocityProblem({0.0, 0.0, 0.0}, {1.0, 0.2, 0.3}), [](Point2, double) { return false; },
                           40);
  EXPECT_NEAR(run.velocity.vx, 1.0, 0.1);
  EXPECT_NEAR(run.velocity.vy, 0.2, 0.1);
  EXPECT_NEAR(run.velocity.wz, 0.3, 0.1);
}

TEST(Mppi, VelocityModeStopsInFrontOfAWall) {
  Mppi mppi;
  const World world = [](Point2 p, double) { return p.x > 2.0; };
  MppiProblem problem = VelocityProblem({0.0, 0.0, 0.0}, {1.0, 0.0, 0.0});
  problem.padding = 0.2;
  const Outcome run = Simulate(mppi, problem, world, 120);
  EXPECT_GT(run.min_clearance, 0.0);
  EXPECT_NEAR(run.velocity.vx, 0.0, 0.1);
  // Stopped near the padding, not far short of it: the front circle's edge is at x + 0.35 + 0.28.
  const double gap = 2.0 - (run.pose.x + 0.63);
  EXPECT_GT(gap, 0.5 * problem.padding);
  EXPECT_LT(gap, problem.padding + 0.4);
}

TEST(Mppi, VelocityModeSlidesAlongAWall) {
  Mppi mppi;
  const World world = [](Point2 p, double) { return p.x > 1.5; };
  const Outcome first = Simulate(mppi, VelocityProblem({0.0, 0.0, 0.0}, {0.8, 0.4, 0.0}), world, 100);
  EXPECT_GT(first.min_clearance, 0.0);
  // Blocked in x, still moving sideways.
  EXPECT_NEAR(first.velocity.vx, 0.0, 0.15);
  EXPECT_GT(first.velocity.vy, 0.25);
  EXPECT_GT(first.pose.y, 1.0);
}

TEST(Mppi, AvoidsAMovingObstacle) {
  Mppi mppi;
  // A 0.6 m box crossing the robot's path at x = 0, at 0.5 m/s.
  const World world = [](Point2 p, double t) { return Box(p, {0.0, 2.5 - 0.5 * t}, 0.3, 0.3); };
  const Outcome run = Simulate(mppi, GoalProblem({-2.5, 0.0, 0.0}, {2.5, 0.0, 0.0}), world, 500, /*dynamic=*/true);
  EXPECT_TRUE(run.arrived) << run.pose.x << ", " << run.pose.y;
  EXPECT_GT(run.min_clearance, 0.0);
}

TEST(Mppi, IsDeterministic) {
  const World world = [](Point2 p, double) { return Box(p, {0.0, 0.3}, 0.4, 0.4); };
  Mppi a, b;
  const Outcome ra = Simulate(a, GoalProblem({-2.0, 0.0, 0.0}, {2.0, 0.0, 0.0}), world, 60);
  const Outcome rb = Simulate(b, GoalProblem({-2.0, 0.0, 0.0}, {2.0, 0.0, 0.0}), world, 60);
  EXPECT_EQ(ra.pose.x, rb.pose.x);
  EXPECT_EQ(ra.pose.y, rb.pose.y);
  EXPECT_EQ(ra.velocity.wz, rb.velocity.wz);
}

TEST(Mppi, PlanningCycleFitsTheBudget) {
  const World world = [](Point2 p, double) {
    return Box(p, {1.0, 0.5}, 0.3, 0.3) || Box(p, {2.0, -1.0}, 0.5, 0.2) || std::abs(p.y - 2.5) < 0.1;
  };
  const Layers layers = Build(world, 0.0, {0.0, 0.0}, {3.0, 0.0});
  Mppi mppi;
  MppiProblem problem = GoalProblem({0.0, 0.0, 0.0}, {3.0, 0.0, 0.0});
  problem.distance = &layers.distance;
  problem.cost_to_go = &layers.cost_to_go;
  mppi.Plan(problem);  // warm up
  constexpr int kCycles = 20;
  const auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < kCycles; ++i) mppi.Plan(problem);
  const double ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count() / kCycles;
  std::printf("MPPI cycle (%d samples x %d steps): %.2f ms\n", mppi.params().samples, mppi.params().horizon_steps, ms);
  // 20 Hz replanning; leave headroom for the map work and the simulation.
  EXPECT_LT(ms, 25.0);

  const auto map_start = std::chrono::steady_clock::now();
  for (int i = 0; i < 5; ++i) Build(world, 0.0, {0.0, 0.0}, {3.0, 0.0});
  const double map_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - map_start).count() / 5;
  std::printf("Map layers (traversability, distance, cost-to-go; 160 x 160): %.2f ms\n", map_ms);
}
