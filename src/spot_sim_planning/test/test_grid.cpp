#include <gtest/gtest.h>

#include <cmath>

#include "spot_sim_planning/grid.hpp"

using spot_sim::planning::CellIndex;
using spot_sim::planning::Grid;
using spot_sim::planning::GridGeometry;
using spot_sim::planning::Interpolate;
using spot_sim::planning::Point2;

TEST(Grid, CellsAndCentres) {
  const GridGeometry g(0.5, 4, 3, Point2{-1.0, 2.0});
  EXPECT_EQ(g.cell_count(), 12u);
  EXPECT_EQ(*g.CellAt({-1.0, 2.0}), (CellIndex{0, 0}));
  EXPECT_EQ(*g.CellAt({0.99, 3.49}), (CellIndex{3, 2}));
  EXPECT_FALSE(g.CellAt({1.0, 2.0}));
  EXPECT_FALSE(g.CellAt({-1.01, 2.0}));
  EXPECT_DOUBLE_EQ(g.Center({1, 2}).x, -0.25);
  EXPECT_DOUBLE_EQ(g.Center({1, 2}).y, 3.25);
  EXPECT_DOUBLE_EQ(g.center().x, 0.0);
  EXPECT_DOUBLE_EQ(g.center().y, 2.75);
  for (std::size_t i = 0; i < g.cell_count(); ++i) {
    EXPECT_EQ(g.Index(g.FromIndex(i)), i);
  }
}

TEST(Grid, InterpolatesBetweenCellCentres) {
  Grid<float> grid(GridGeometry(1.0, 3, 3, Point2{}), 0.0f);
  for (std::size_t i = 0; i < grid.size(); ++i) {
    const CellIndex c = grid.geometry().FromIndex(i);
    grid[i] = static_cast<float>(2.0 * c.x + 3.0 * c.y);  // a plane
  }
  // Cell centres sit at 0.5, 1.5, 2.5.
  EXPECT_NEAR(*Interpolate(grid, {1.0, 1.0}), 2.0 * 0.5 + 3.0 * 0.5, 1e-9);
  EXPECT_NEAR(*Interpolate(grid, {1.5, 2.2}), 2.0 * 1.0 + 3.0 * 1.7, 1e-6);
  EXPECT_FALSE(Interpolate(grid, {0.2, 1.0}));  // before the first centre
  grid.at({1, 1}) = NAN;
  EXPECT_FALSE(Interpolate(grid, {1.0, 1.0}));
}
