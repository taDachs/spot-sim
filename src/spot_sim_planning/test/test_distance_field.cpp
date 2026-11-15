#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "spot_sim_planning/distance_field.hpp"

using spot_sim::planning::CellIndex;
using spot_sim::planning::Grid;
using spot_sim::planning::GridGeometry;
using spot_sim::planning::Point2;
using spot_sim::planning::SignedDistance;

namespace {
/// Distance from `c` to the nearest cell with mask == target, by brute force.
double BruteForce(const Grid<uint8_t>& mask, CellIndex c, uint8_t target) {
  const GridGeometry& g = mask.geometry();
  double best = INFINITY;
  for (std::size_t i = 0; i < mask.size(); ++i) {
    if (mask[i] != target) continue;
    const CellIndex o = g.FromIndex(i);
    best = std::min(best, std::hypot(o.x - c.x, o.y - c.y) * g.resolution());
  }
  return best;
}
}  // namespace

TEST(DistanceField, MatchesBruteForce) {
  std::mt19937 rng(7);
  std::bernoulli_distribution obstacle(0.08);
  Grid<uint8_t> mask(GridGeometry(0.05, 37, 23, Point2{}), 0);
  for (std::size_t i = 0; i < mask.size(); ++i) mask[i] = obstacle(rng) ? 1 : 0;
  const auto d = SignedDistance(mask);
  for (std::size_t i = 0; i < mask.size(); ++i) {
    const CellIndex c = mask.geometry().FromIndex(i);
    const double expected = mask[i] ? -BruteForce(mask, c, 0) : BruteForce(mask, c, 1);
    ASSERT_NEAR(d[i], expected, 1e-5) << c.x << ", " << c.y;
  }
}

TEST(DistanceField, Extremes) {
  const Grid<uint8_t> free(GridGeometry(0.1, 5, 5, Point2{}), 0);
  const auto free_distance = SignedDistance(free);
  for (const float v : free_distance.data()) EXPECT_EQ(v, INFINITY);
  const Grid<uint8_t> full(GridGeometry(0.1, 5, 5, Point2{}), 1);
  const auto full_distance = SignedDistance(full);
  for (const float v : full_distance.data()) EXPECT_EQ(v, -INFINITY);
}

TEST(DistanceField, Wall) {
  Grid<uint8_t> mask(GridGeometry(0.1, 20, 10, Point2{}), 0);
  for (int y = 0; y < 10; ++y) mask.at({10, y}) = 1;
  const auto d = SignedDistance(mask);
  EXPECT_NEAR(d.at({2, 5}), 0.8, 1e-6);
  EXPECT_NEAR(d.at({13, 0}), 0.3, 1e-6);
  EXPECT_NEAR(d.at({10, 3}), -0.1, 1e-6);
}
