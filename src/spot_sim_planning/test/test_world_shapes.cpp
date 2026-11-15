#include <gtest/gtest.h>

#include <cmath>

#include "spot_sim_planning/traversability.hpp"
#include "spot_sim_planning/world_shapes.hpp"

using spot_sim::planning::CellIndex;
using spot_sim::planning::ComputeTraversability;
using spot_sim::planning::DrawMovable;
using spot_sim::planning::FillStatic;
using spot_sim::planning::FootprintBounds;
using spot_sim::planning::HeightMap;
using spot_sim::planning::kNoSurface;
using spot_sim::planning::Point2;
using spot_sim::planning::Quat;
using spot_sim::planning::Shape;
using spot_sim::planning::TopSurface;
using spot_sim::planning::Vec3;

namespace {
constexpr double kPi = 3.14159265358979323846;

Shape Make(Shape::Kind kind, Vec3 position, Vec3 size, Quat orientation = {}) {
  Shape s;
  s.kind = kind;
  s.position = position;
  s.size = size;
  s.orientation = orientation;
  return s;
}

Shape Ground() { return Make(Shape::Kind::kPlane, {0, 0, 0}, {100, 100, 0}); }
}  // namespace

TEST(WorldShapes, Box) {
  const Shape box = Make(Shape::Kind::kBox, {1.0, 0.0, 0.4}, {0.8, 0.8, 0.8});
  EXPECT_NEAR(*TopSurface(box, 1.2, 0.3, 2.0), 0.8, 1e-9);
  EXPECT_FALSE(TopSurface(box, 1.5, 0.0, 2.0));
  EXPECT_NEAR(*TopSurface(box, 1.0, 0.0, 0.5), 0.5, 1e-9);  // starts inside
  // Rotated 45 deg about z: (1.5, 0) is inside the diamond's tip region.
  const Shape diamond = Make(Shape::Kind::kBox, {1.0, 0.0, 0.4}, {0.8, 0.8, 0.8}, Quat::FromRpy(0, 0, kPi / 4));
  EXPECT_NEAR(*TopSurface(diamond, 1.5, 0.0, 2.0), 0.8, 1e-9);
  EXPECT_FALSE(TopSurface(diamond, 1.35, 0.35, 2.0));
  // Tilted 30 deg about x: a ramp-like top.
  const Shape tilted = Make(Shape::Kind::kBox, {0.0, 0.0, 0.0}, {1.0, 1.0, 0.2}, Quat::FromRpy(kPi / 6, 0, 0));
  const double z0 = *TopSurface(tilted, 0.0, 0.0, 2.0), z1 = *TopSurface(tilted, 0.0, 0.2, 2.0);
  EXPECT_NEAR(z1 - z0, 0.2 * std::tan(kPi / 6), 1e-9);
}

TEST(WorldShapes, RoundShapes) {
  const Shape sphere = Make(Shape::Kind::kSphere, {0.0, 0.0, 0.5}, {0.5, 0, 0});
  EXPECT_NEAR(*TopSurface(sphere, 0.0, 0.0, 2.0), 1.0, 1e-9);
  EXPECT_NEAR(*TopSurface(sphere, 0.3, 0.0, 2.0), 0.5 + 0.4, 1e-9);
  EXPECT_FALSE(TopSurface(sphere, 0.6, 0.0, 2.0));

  const Shape upright = Make(Shape::Kind::kCylinder, {0.0, 0.0, 0.5}, {0.3, 0, 1.0});
  EXPECT_NEAR(*TopSurface(upright, 0.2, 0.1, 2.0), 1.0, 1e-9);
  EXPECT_FALSE(TopSurface(upright, 0.25, 0.25, 2.0));
  // Lying along x (rotated 90 deg about y), radius 0.3, length 1.
  const Shape lying = Make(Shape::Kind::kCylinder, {0.0, 0.0, 0.3}, {0.3, 0, 1.0}, Quat::FromRpy(0, kPi / 2, 0));
  EXPECT_NEAR(*TopSurface(lying, 0.4, 0.0, 2.0), 0.6, 1e-9);
  EXPECT_NEAR(*TopSurface(lying, 0.0, 0.18, 2.0), 0.3 + 0.24, 1e-9);
  EXPECT_FALSE(TopSurface(lying, 0.6, 0.0, 2.0));
  // A capsule adds hemispheres at the ends.
  const Shape capsule = Make(Shape::Kind::kCapsule, {0.0, 0.0, 0.3}, {0.3, 0, 1.0}, Quat::FromRpy(0, kPi / 2, 0));
  EXPECT_NEAR(*TopSurface(capsule, 0.6, 0.0, 2.0), 0.3 + 0.3 * std::sqrt(1 - (0.1 / 0.3) * (0.1 / 0.3)), 1e-9);
}

TEST(WorldShapes, Plane) {
  EXPECT_NEAR(*TopSurface(Ground(), 3.0, -2.0, 1.0), 0.0, 1e-9);
  EXPECT_FALSE(TopSurface(Ground(), 51.0, 0.0, 1.0));
  const Shape ramp = Make(Shape::Kind::kPlane, {0, 0, 0}, {4, 4, 0}, Quat::FromRpy(0, -0.2, 0));
  EXPECT_NEAR(*TopSurface(ramp, 1.0, 0.0, 2.0), std::tan(0.2), 1e-9);
}

TEST(WorldShapes, FootprintBounds) {
  const auto b = FootprintBounds(Make(Shape::Kind::kBox, {1, 2, 0}, {2, 1, 1}, Quat::FromRpy(0, 0, kPi / 2)));
  EXPECT_NEAR(b.min_x, 0.5, 1e-9);
  EXPECT_NEAR(b.max_x, 1.5, 1e-9);
  EXPECT_NEAR(b.min_y, 1.0, 1e-9);
  EXPECT_NEAR(b.max_y, 3.0, 1e-9);
}

TEST(WorldShapes, FillStaticAndDrawMovable) {
  HeightMap map(80, 0.05);  // 4 x 4 m around the origin
  const std::vector<Shape> statics = {
      Ground(),
      Make(Shape::Kind::kBox, {1.0, 0.0, 0.4}, {0.8, 0.8, 0.8}),
      Make(Shape::Kind::kBox, {-1.0, 1.0, 2.0}, {1.0, 1.0, 0.1}),  // a roof well above: ignored
  };
  EXPECT_EQ(FillStatic(map, statics, 1.0), 80u * 80u);
  EXPECT_TRUE(map.UnknownStaticCells().empty());
  EXPECT_NEAR(*map.HeightAt({1.0, 0.0}), 0.8, 1e-6);
  EXPECT_NEAR(*map.HeightAt({-1.0, 1.0}), 0.0, 1e-6);
  EXPECT_NEAR(*map.HeightAt({0.0, -1.0}), 0.0, 1e-6);
  EXPECT_EQ(FillStatic(map, statics, 1.0), 0u);  // nothing left to fill

  DrawMovable(map, {Make(Shape::Kind::kBox, {-1.0, -1.0, 0.15}, {0.3, 0.3, 0.3})}, 1.0);
  EXPECT_NEAR(*map.HeightAt({-1.0, -1.0}), 0.3, 1e-6);
  DrawMovable(map, {Make(Shape::Kind::kBox, {0.0, -1.0, 0.15}, {0.3, 0.3, 0.3})}, 1.0);  // it moved
  EXPECT_NEAR(*map.HeightAt({-1.0, -1.0}), 0.0, 1e-6);
  EXPECT_NEAR(*map.HeightAt({0.0, -1.0}), 0.3, 1e-6);

  const auto t = ComputeTraversability(map.Heights());
  EXPECT_EQ(t.obstacle.at(*map.geometry().CellAt({0.61, 0.0})), 1);  // the static box's edge
  EXPECT_EQ(t.obstacle.at(*map.geometry().CellAt({-1.5, 1.5})), 0);
}

TEST(WorldShapes, NoSurfaceIsAnObstacle) {
  HeightMap map(40, 0.05);
  // Ground only for x < 0.5: beyond is a drop into nothing.
  FillStatic(map, {Make(Shape::Kind::kPlane, {-4.75, 0, 0}, {10.5, 10, 0})}, 1.0);
  EXPECT_EQ(map.Static(*map.geometry().CellAt({0.8, 0.0})), kNoSurface);
  const auto t = ComputeTraversability(map.Heights());
  EXPECT_EQ(t.obstacle.at(*map.geometry().CellAt({0.8, 0.0})), 1);
  EXPECT_EQ(t.obstacle.at(*map.geometry().CellAt({0.47, 0.0})), 1);  // the edge
  EXPECT_EQ(t.obstacle.at(*map.geometry().CellAt({0.0, 0.0})), 0);
}
