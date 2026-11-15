#pragma once

#include <limits>
#include <optional>
#include <vector>

#include "spot_sim_planning/height_map.hpp"

namespace spot_sim::planning {

struct Vec3 {
  double x = 0.0, y = 0.0, z = 0.0;
  Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
  Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
  Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
};

/// Unit quaternion (w, x, y, z).
struct Quat {
  double w = 1.0, x = 0.0, y = 0.0, z = 0.0;
  Vec3 Rotate(const Vec3& v) const;
  Quat Conjugate() const { return {w, -x, -y, -z}; }
  static Quat FromRpy(double roll, double pitch, double yaw);
};

/// A collision shape of the world, in the map frame (gravity-aligned, z up).
/// Other geometry (meshes, ellipsoids, ...) is represented by its bounding box.
struct Shape {
  enum class Kind { kBox, kSphere, kCylinder, kCapsule, kPlane };
  Kind kind = Kind::kBox;
  Vec3 position;
  Quat orientation;
  /// kBox: full edge lengths. kSphere: x = radius. kCylinder, kCapsule:
  /// x = radius, z = length of the cylindrical part (along the local z axis).
  /// kPlane: x, y = extent in the local x / y directions (normal = local z).
  Vec3 size;
};

/// Axis-aligned bounds of a shape's footprint in the map plane.
struct Bounds2 {
  double min_x = 0.0, min_y = 0.0, max_x = 0.0, max_y = 0.0;
};
Bounds2 FootprintBounds(const Shape& shape);

/// Where a ray straight down from (x, y, z_top) first meets `shape`: the z of
/// its top surface there; z_top if the ray starts inside it; empty if it
/// misses.
std::optional<double> TopSurface(const Shape& shape, double x, double y, double z_top);

/// No surface below a cell (e.g. beyond the edge of the ground): the cell is
/// known, and impassable (-inf makes the step rule fire).
inline constexpr float kNoSurface = -std::numeric_limits<float>::infinity();

/// Fills the map's unknown static cells: the highest top surface of `shapes`
/// below `z_top` at each cell centre, kNoSurface where nothing is below.
/// Things entirely above `z_top` (e.g. a table the robot walks under) are
/// ignored. Returns the number of cells filled.
std::size_t FillStatic(HeightMap& map, const std::vector<Shape>& shapes, double z_top);

/// Redraws the movable layer from `shapes` (only the cells under each shape).
void DrawMovable(HeightMap& map, const std::vector<Shape>& shapes, double z_top);

}  // namespace spot_sim::planning
