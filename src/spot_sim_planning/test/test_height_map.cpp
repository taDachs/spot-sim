#include <gtest/gtest.h>

#include <cmath>

#include "spot_sim_planning/height_map.hpp"

using spot_sim::planning::CellIndex;
using spot_sim::planning::HeightMap;
using spot_sim::planning::Point2;

namespace {
/// Fills the static layer with a function of the cell centre.
template <typename Fn>
void FillStatic(HeightMap& map, Fn height) {
  for (const CellIndex c : map.UnknownStaticCells()) {
    map.SetStatic(c, static_cast<float>(height(map.geometry().Center(c))));
  }
}
}  // namespace

TEST(HeightMap, StartsUnknownAndCentredOnTheOrigin) {
  HeightMap map(160, 0.05);
  EXPECT_EQ(map.UnknownStaticCells().size(), 160u * 160u);
  EXPECT_NEAR(map.geometry().center().x, 0.0, 0.05);
  EXPECT_NEAR(map.geometry().center().y, 0.0, 0.05);
  EXPECT_TRUE(map.geometry().CellAt({0.0, 0.0}));
  EXPECT_TRUE(std::isnan(map.Height({10, 10})));
  EXPECT_FALSE(map.HeightAt({0.0, 0.0}));
}

TEST(HeightMap, RollingKeepsCellsInPlace) {
  HeightMap map(40, 0.1);  // 4 x 4 m
  FillStatic(map, [](Point2 p) { return p.x + 10.0 * p.y; });
  ASSERT_TRUE(map.UnknownStaticCells().empty());
  const double before = *map.HeightAt({0.73, -0.41});

  EXPECT_TRUE(map.Recenter({1.03, -0.52}));
  EXPECT_NEAR(map.geometry().center().x, 1.0, 0.1);
  EXPECT_NEAR(map.geometry().center().y, -0.55, 0.06);
  // The window lies on the same lattice: origins are multiples of the resolution.
  EXPECT_NEAR(std::remainder(map.geometry().origin().x, 0.1), 0.0, 1e-9);
  EXPECT_NEAR(std::remainder(map.geometry().origin().y, 0.1), 0.0, 1e-9);
  // Still-covered places keep their heights; newly covered cells are unknown.
  EXPECT_NEAR(*map.HeightAt({0.73, -0.41}), before, 1e-5);
  const auto unknown = map.UnknownStaticCells();
  EXPECT_EQ(unknown.size(), 40u * 40u - 30u * 34u);  // moved 10 cells in x, 6 in y
  for (const CellIndex c : unknown) {
    const Point2 p = map.geometry().Center(c);
    EXPECT_TRUE(p.x > 2.0 || p.y < -2.0) << p.x << ", " << p.y;
  }
  FillStatic(map, [](Point2 p) { return p.x + 10.0 * p.y; });
  EXPECT_NEAR(*map.HeightAt({2.6, -2.3}), 2.6 + 10.0 * -2.3, 1e-4);

  EXPECT_FALSE(map.Recenter({1.03, -0.52}));  // already there
}

TEST(HeightMap, RecenterIfNeededHasSlack) {
  HeightMap map(40, 0.1);
  EXPECT_FALSE(map.RecenterIfNeeded({0.9, -0.9}, 1.0));
  EXPECT_TRUE(map.RecenterIfNeeded({1.2, 0.0}, 1.0));
  EXPECT_NEAR(map.geometry().center().x, 1.2, 0.1);
}

TEST(HeightMap, MovableLayerIsRedrawnAndTheHigherLayerWins) {
  HeightMap map(20, 0.1);
  FillStatic(map, [](Point2) { return 0.0; });
  const CellIndex c{5, 5};
  map.SetMovable(c, 0.3f);
  map.SetMovable(c, 0.2f);  // keeps the highest surface
  EXPECT_FLOAT_EQ(map.Height(c), 0.3f);
  map.SetStatic(c, 0.5f);
  EXPECT_FLOAT_EQ(map.Height(c), 0.5f);
  map.ClearMovable();
  map.SetStatic(c, 0.0f);
  EXPECT_FLOAT_EQ(map.Height(c), 0.0f);

  // Only movable data known: still a height.
  map.InvalidateStatic();
  map.SetMovable(c, 0.4f);
  EXPECT_FLOAT_EQ(map.Height(c), 0.4f);
  EXPECT_TRUE(std::isnan(map.Height({6, 5})));
}

TEST(HeightMap, HeightsGridMatchesCells) {
  HeightMap map(10, 0.1);
  FillStatic(map, [](Point2 p) { return p.x; });
  map.SetMovable({3, 4}, 2.0f);
  const auto heights = map.Heights();
  EXPECT_FLOAT_EQ(heights.at({3, 4}), 2.0f);
  EXPECT_FLOAT_EQ(heights.at({7, 1}), map.Static({7, 1}));
}
