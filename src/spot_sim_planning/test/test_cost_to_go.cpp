#include <gtest/gtest.h>

#include <cmath>

#include "spot_sim_planning/cost_to_go.hpp"
#include "spot_sim_planning/distance_field.hpp"

using spot_sim::planning::CellIndex;
using spot_sim::planning::ComputeCostToGo;
using spot_sim::planning::ComputeTraversability;
using spot_sim::planning::CostToGoParams;
using spot_sim::planning::Grid;
using spot_sim::planning::GridGeometry;
using spot_sim::planning::Point2;
using spot_sim::planning::SignedDistance;

namespace {
/// 8 x 8 m at 5 cm centred on the origin, flat except where `wall` is true
/// (1 m high).
template <typename Fn>
Grid<float> CostToGo(Fn wall, Point2 goal, CostToGoParams params = {}) {
  Grid<float> heights(GridGeometry(0.05, 160, 160, Point2{-4.0, -4.0}), 0.0f);
  for (std::size_t i = 0; i < heights.size(); ++i) {
    heights[i] = wall(heights.geometry().Center(heights.geometry().FromIndex(i))) ? 1.0f : 0.0f;
  }
  const auto t = ComputeTraversability(heights);
  return ComputeCostToGo(t, SignedDistance(t.obstacle), goal, params);
}

double At(const Grid<float>& field, Point2 p) { return field.at(*field.geometry().CellAt(p)); }
}  // namespace

TEST(CostToGo, FreeSpaceIsAboutTheDistance) {
  const auto field = CostToGo([](Point2) { return false; }, {1.0, 1.0});
  EXPECT_NEAR(At(field, {1.0, 1.0}), 0.0, 1e-6);
  // 8-connected paths are at most ~8% longer than straight lines.
  for (const Point2 p : {Point2{-3.0, 1.0}, Point2{-2.0, -3.0}, Point2{3.5, -0.5}}) {
    const double straight = (p - Point2{1.0, 1.0}).Norm();
    EXPECT_GE(At(field, p), straight - 0.08);
    EXPECT_LE(At(field, p), straight * 1.09);
  }
}

TEST(CostToGo, GoesThroughTheGap) {
  // Wall along x = 0 with a 1.2 m gap at y in [2, 3.2].
  const auto wall = [](Point2 p) { return std::abs(p.x) < 0.1 && !(p.y > 2.0 && p.y < 3.2); };
  const auto field = CostToGo(wall, {2.0, 0.0});
  const double cost = At(field, {-2.0, 0.0});
  ASSERT_TRUE(std::isfinite(cost));
  // Via the gap's middle (0, 2.6): 2 * hypot(2, 2.6) = 6.56, plus some room near the walls.
  EXPECT_GT(cost, 6.4);
  EXPECT_LT(cost, 9.0);
  // Downhill from the start leads toward the gap, not into the wall.
  EXPECT_LT(At(field, {-1.0, 1.5}), At(field, {-1.0, -1.5}));
}

TEST(CostToGo, ClosedWallAndNarrowGapsBlock) {
  const auto closed = CostToGo([](Point2 p) { return std::abs(p.x) < 0.1; }, {2.0, 0.0});
  EXPECT_FALSE(std::isfinite(At(closed, {-2.0, 0.0})));
  EXPECT_TRUE(std::isfinite(At(closed, {3.0, 3.0})));
  // A 0.5 m gap is narrower than twice the inflation radius (0.35 m).
  const auto narrow =
      CostToGo([](Point2 p) { return std::abs(p.x) < 0.1 && !(p.y > 1.0 && p.y < 1.5); }, {2.0, 0.0});
  EXPECT_FALSE(std::isfinite(At(narrow, {-2.0, 0.0})));
}

TEST(CostToGo, UnreachableGoals) {
  // Inside a wall: the goal cell itself is impassable.
  const auto in_wall = CostToGo([](Point2 p) { return std::abs(p.x) < 0.1; }, {0.0, 0.0});
  for (const float v : in_wall.data()) ASSERT_FALSE(std::isfinite(v));
  // On top of a 1 m high block: its flat top is walkable, but cut off from the ground.
  const auto on_block = CostToGo([](Point2 p) { return p.Norm() < 0.6; }, {0.0, 0.0});
  EXPECT_FALSE(std::isfinite(At(on_block, {2.0, 0.0})));
}

TEST(CostToGo, GoalOutsideTheWindowGuidesTowardIt) {
  const auto field = CostToGo([](Point2) { return false; }, {20.0, 0.0});
  EXPECT_NEAR(At(field, {3.9, 0.0}), 16.1, 0.1);
  EXPECT_NEAR(At(field, {0.0, 0.0}), 20.0, 0.4);
  EXPECT_LT(At(field, {1.0, 0.0}), At(field, {0.0, 0.0}));
  EXPECT_LT(At(field, {0.0, 0.0}), At(field, {0.0, 2.0}));
}

TEST(CostToGo, ProximityCostKeepsRoom) {
  // A corridor 2 m wide (walls at y = +-1): staying central is cheaper.
  const auto wall = [](Point2 p) { return std::abs(p.y) > 1.0; };
  const auto field = CostToGo(wall, {3.0, 0.0});
  EXPECT_LT(At(field, {-3.0, 0.0}), At(field, {-3.0, 0.5}));
}
