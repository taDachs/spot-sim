#include <gtest/gtest.h>

#include <cmath>
#include <functional>
#include <memory>

#include "spot_sim_planning/local_planner.hpp"

using spot_sim::planning::Integrate;
using spot_sim::planning::LocalPlanner;
using spot_sim::planning::LocalPlannerInput;
using spot_sim::planning::LocalPlannerOutput;
using spot_sim::planning::LocalPlannerState;
using spot_sim::planning::Pose2;
using spot_sim::planning::Quat;
using spot_sim::planning::Shape;
using spot_sim::planning::Vec3;
using spot_sim::planning::Velocity2;

namespace {

Shape Box(Vec3 center, Vec3 size, double yaw = 0.0) {
  Shape s;
  s.kind = Shape::Kind::kBox;
  s.position = center;
  s.size = size;
  s.orientation = Quat::FromRpy(0, 0, yaw);
  return s;
}

std::shared_ptr<const std::vector<Shape>> World(std::vector<Shape> shapes) {
  Shape ground;
  ground.kind = Shape::Kind::kPlane;
  ground.size = {100, 100, 0};
  shapes.push_back(ground);
  return std::make_shared<const std::vector<Shape>>(std::move(shapes));
}

/// Runs the planner in closed loop (the robot follows each command exactly)
/// until `done` or `max_steps`. `movable(t)` gives the movable shapes.
struct Loop {
  LocalPlanner planner;
  LocalPlannerInput input;
  LocalPlannerOutput out;
  std::function<std::vector<Shape>(double)> movable = [](double) { return std::vector<Shape>{}; };

  int Run(int max_steps, const std::function<bool(const LocalPlannerOutput&)>& done) {
    const double dt = planner.config().mppi.dt;
    for (int step = 0; step < max_steps; ++step) {
      input.time = step * dt;
      input.movable_shapes = movable(input.time);
      out = planner.Step(input);
      input.current = out.command;
      input.robot = Integrate(input.robot, out.command, dt);
      if (done(out)) return step + 1;
    }
    return max_steps;
  }
};

LocalPlannerInput GoalInput(Pose2 start, Pose2 goal, std::shared_ptr<const std::vector<Shape>> world) {
  LocalPlannerInput input;
  input.mode = LocalPlannerInput::Mode::kGoal;
  input.robot = start;
  input.robot_z = 0.52;
  input.goal = goal;
  input.goal_sequence = 1;
  input.static_shapes = std::move(world);
  input.static_version = 1;
  return input;
}

bool At(const LocalPlannerOutput& out, LocalPlannerState state) { return out.state == state; }

}  // namespace

TEST(LocalPlanner, ReachesTheGoalAroundABoxAndStops) {
  Loop loop;
  loop.input = GoalInput({-2.5, 0.0, 0.0}, {2.5, 0.0, 0.0}, World({Box({0, 0, 0.5}, {0.8, 0.8, 1.0})}));
  const int steps = loop.Run(600, [](const auto& out) { return At(out, LocalPlannerState::kAtGoal); });
  ASSERT_LT(steps, 600);
  EXPECT_NEAR(loop.input.robot.x, 2.5, 0.1);
  EXPECT_NEAR(loop.input.robot.y, 0.0, 0.1);
  EXPECT_EQ(loop.out.command.vx, 0.0);
  // Stays there.
  loop.Run(20, [](const auto&) { return false; });
  EXPECT_EQ(loop.out.state, LocalPlannerState::kAtGoal);
}

TEST(LocalPlanner, CrossesTheMapWindow) {
  Loop loop;
  loop.input = GoalInput({0.0, 0.0, 0.0}, {9.0, 1.0, 0.0}, World({Box({4.0, 0.5, 0.5}, {0.6, 2.0, 1.0})}));
  const int steps = loop.Run(1200, [](const auto& out) { return At(out, LocalPlannerState::kAtGoal); });
  EXPECT_LT(steps, 1200) << loop.input.robot.x << ", " << loop.input.robot.y;
  EXPECT_GT(loop.planner.map().geometry().center().x, 7.0);  // the window followed
}

TEST(LocalPlanner, BlockedBehindAClosedWallAndRecoversWhenItOpens) {
  Loop loop;
  loop.input = GoalInput({-2.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, World({}));
  // A movable wall across x = 0 that disappears after 4 s.
  loop.movable = [](double t) {
    return t < 4.0 ? std::vector<Shape>{Box({0, 0, 0.5}, {0.2, 20.0, 1.0})} : std::vector<Shape>{};
  };
  const int blocked = loop.Run(80, [](const auto& out) { return At(out, LocalPlannerState::kBlocked); });
  EXPECT_LT(blocked, 80);  // within 4 s
  EXPECT_LT(loop.input.robot.x, -0.3);
  // Keeps trying; once the wall is gone it gets there.
  const int arrived = loop.Run(600, [](const auto& out) { return At(out, LocalPlannerState::kAtGoal); });
  EXPECT_LT(arrived, 600);
}

TEST(LocalPlanner, NewGoalStartsOver) {
  Loop loop;
  loop.input = GoalInput({0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, World({}));
  loop.Run(5, [](const auto&) { return false; });
  EXPECT_EQ(loop.out.state, LocalPlannerState::kAtGoal);
  loop.input.goal = {1.5, 0.0, 0.0};
  loop.input.goal_sequence = 2;
  loop.Run(1, [](const auto&) { return false; });
  EXPECT_NE(loop.out.state, LocalPlannerState::kAtGoal);
  EXPECT_LT(loop.Run(300, [](const auto& out) { return At(out, LocalPlannerState::kAtGoal); }), 300);
}

TEST(LocalPlanner, VelocityModeTracksAndStopsAtObstacles) {
  Loop loop;
  loop.input.mode = LocalPlannerInput::Mode::kVelocity;
  loop.input.robot_z = 0.52;
  loop.input.commanded = {0.8, 0.0, 0.0};
  loop.input.static_shapes = World({Box({5.0, 0, 0.5}, {0.2, 4.0, 1.0})});
  loop.input.static_version = 1;
  loop.Run(30, [](const auto&) { return false; });
  EXPECT_NEAR(loop.out.command.vx, 0.8, 0.1);
  loop.Run(200, [](const auto&) { return false; });
  EXPECT_NEAR(loop.out.command.vx, 0.0, 0.1);
  EXPECT_LT(loop.input.robot.x + 0.63, 4.9);  // front of the footprint before the wall
  EXPECT_GT(loop.input.robot.x + 0.63, 4.3);
}

TEST(LocalPlanner, TallThingsAboveTheRobotDontBlock) {
  // A table top 1 m up: the robot walks under it.
  Loop loop;
  loop.input = GoalInput({-2.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, World({Box({0, 0, 1.0}, {1.0, 1.0, 0.05})}));
  EXPECT_LT(loop.Run(400, [](const auto& out) { return At(out, LocalPlannerState::kAtGoal); }), 400);
  EXPECT_LT(std::abs(loop.input.robot.y), 0.3);  // straight through
}

TEST(LocalPlanner, WithoutAvoidanceGoesStraightThrough) {
  Loop loop;
  loop.input = GoalInput({-2.5, 0.0, 0.0}, {2.5, 0.0, 0.0}, World({Box({0, 0, 0.5}, {0.8, 0.8, 1.0})}));
  loop.input.avoid = false;
  double closest = INFINITY;
  const int steps = loop.Run(400, [&](const auto& out) {
    closest = std::min(closest, std::hypot(loop.input.robot.x, loop.input.robot.y));
    return At(out, LocalPlannerState::kAtGoal);
  });
  EXPECT_LT(steps, 400);
  EXPECT_LT(closest, 0.2);  // right through the box
}

TEST(LocalPlanner, YardAroundTheRedBox) {
  // The yard world's objects (yard.sdf).
  Shape cylinder;
  cylinder.kind = Shape::Kind::kCylinder;
  cylinder.position = {0.0, 2.0, 0.5};
  cylinder.size = {0.3, 0, 1.0};
  Shape sphere;
  sphere.kind = Shape::Kind::kSphere;
  sphere.position = {0.5, -2.0, 0.35};
  sphere.size = {0.35, 0, 0};
  const auto yard = World({Box({3.2, 0.0, 0.4}, {0.8, 0.8, 0.8}, 0.3), cylinder, Box({-3.0, 0.0, 0.75}, {0.2, 4.0, 1.5}),
                           sphere});
  Loop loop;
  loop.input = GoalInput({0.5, -0.8, 1.57}, {4.7, 0.2, 0.0}, yard);
  int blocked = 0;
  const int steps = loop.Run(800, [&](const auto& out) {
    blocked += out.state == LocalPlannerState::kBlocked;
    return At(out, LocalPlannerState::kAtGoal);
  });
  EXPECT_LT(steps, 800) << loop.input.robot.x << ", " << loop.input.robot.y << ", " << loop.input.robot.yaw;
  EXPECT_EQ(blocked, 0);
}
