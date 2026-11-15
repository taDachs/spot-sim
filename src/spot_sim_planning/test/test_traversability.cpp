#include <gtest/gtest.h>

#include <cmath>

#include "spot_sim_planning/traversability.hpp"

using spot_sim::planning::CellIndex;
using spot_sim::planning::ComputeTraversability;
using spot_sim::planning::Grid;
using spot_sim::planning::GridGeometry;
using spot_sim::planning::Point2;
using spot_sim::planning::Traversability;

namespace {
constexpr double kDeg = 3.14159265358979323846 / 180.0;

/// 4 x 4 m at 5 cm, heights from a function of the cell centre.
template <typename Fn>
Grid<float> Terrain(Fn height) {
  Grid<float> grid(GridGeometry(0.05, 80, 80, Point2{-2.0, -2.0}), 0.0f);
  for (std::size_t i = 0; i < grid.size(); ++i) {
    grid[i] = static_cast<float>(height(grid.geometry().Center(grid.geometry().FromIndex(i))));
  }
  return grid;
}

bool Obstacle(const Traversability& t, Point2 p) { return t.obstacle.at(*t.obstacle.geometry().CellAt(p)) != 0; }
}  // namespace

TEST(Traversability, FlatGroundIsFreeAtUnitCost) {
  const auto t = ComputeTraversability(Terrain([](Point2) { return 0.0; }));
  for (std::size_t i = 0; i < t.obstacle.size(); ++i) {
    ASSERT_EQ(t.obstacle[i], 0);
    ASSERT_FLOAT_EQ(t.cost[i], 1.0f);
  }
}

TEST(Traversability, BoxEdgesAreObstaclesItsTopIsNot) {
  // A 30 cm high, 1 m wide box: too high to step onto.
  const auto t = ComputeTraversability(
      Terrain([](Point2 p) { return std::abs(p.x) < 0.5 && std::abs(p.y) < 0.5 ? 0.3 : 0.0; }));
  EXPECT_TRUE(Obstacle(t, {0.49, 0.0}));
  EXPECT_TRUE(Obstacle(t, {0.51, 0.0}));
  EXPECT_TRUE(Obstacle(t, {0.0, -0.51}));
  EXPECT_FALSE(Obstacle(t, {0.0, 0.0}));  // flat top, unreachable from the ground
  EXPECT_FALSE(Obstacle(t, {1.0, 1.0}));
}

TEST(Traversability, StepsUpToTheLimitAreWalkable) {
  const auto low = ComputeTraversability(Terrain([](Point2 p) { return p.x > 0.0 ? 0.15 : 0.0; }));
  EXPECT_FALSE(Obstacle(low, {0.0, 0.0}));
  EXPECT_GT(low.cost.at(*low.cost.geometry().CellAt({0.01, 0.0})), 1.5f);  // steps cost more
  const auto high = ComputeTraversability(Terrain([](Point2 p) { return p.x > 0.0 ? 0.2 : 0.0; }));
  EXPECT_TRUE(Obstacle(high, {0.01, 0.0}));
  EXPECT_TRUE(Obstacle(high, {-0.01, 0.0}));
  EXPECT_FALSE(Obstacle(high, {0.3, 0.0}));
  EXPECT_FALSE(Obstacle(high, {-0.3, 0.0}));
}

TEST(Traversability, SlopesUpToTheLimitAreWalkable) {
  const auto gentle = ComputeTraversability(Terrain([](Point2 p) { return std::tan(20 * kDeg) * p.x; }));
  EXPECT_FALSE(Obstacle(gentle, {0.0, 0.0}));
  EXPECT_NEAR(gentle.slope.at(*gentle.slope.geometry().CellAt({0.0, 0.0})), 20 * kDeg, 0.5 * kDeg);
  const auto steep = ComputeTraversability(Terrain([](Point2 p) { return std::tan(30 * kDeg) * p.x; }));
  EXPECT_TRUE(Obstacle(steep, {0.0, 0.0}));
  // A steep ramp is a plane, not a staircase of small steps.
  const auto very_steep = ComputeTraversability(Terrain([](Point2 p) { return std::tan(60 * kDeg) * p.x; }));
  EXPECT_TRUE(Obstacle(very_steep, {0.0, 0.0}));
}

TEST(Traversability, StairsAreWalkable) {
  // 17 cm risers, 28 cm treads: about 31 deg on average, but a staircase.
  const auto t = ComputeTraversability(
      Terrain([](Point2 p) { return p.x < 0.0 ? 0.0 : 0.17 * (1 + std::floor(p.x / 0.28)); }));
  for (double x = -0.5; x < 1.5; x += 0.05) {
    EXPECT_FALSE(Obstacle(t, {x, 0.0})) << "x = " << x;
  }
}

TEST(Traversability, UnknownCells) {
  auto heights = Terrain([](Point2) { return 0.0; });
  heights.at({10, 10}) = NAN;
  EXPECT_EQ(ComputeTraversability(heights).obstacle.at({10, 10}), 1);
  spot_sim::planning::TraversabilityParams params;
  params.unknown_is_obstacle = false;
  const auto t = ComputeTraversability(heights, params);
  EXPECT_EQ(t.obstacle.at({10, 10}), 0);
  EXPECT_EQ(t.obstacle.at({11, 10}), 0);  // neighbours ignore the unknown cell
}
