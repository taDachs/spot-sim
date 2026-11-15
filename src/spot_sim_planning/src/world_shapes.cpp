#include "spot_sim_planning/world_shapes.hpp"

#include <algorithm>
#include <cmath>

namespace spot_sim::planning {

namespace {

constexpr double kEps = 1e-12;
constexpr double kHuge = 1e9;

double Dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

/// A ray in a shape's local frame: origin `o`, unit direction `d`.
struct Ray {
  Vec3 o, d;
};

/// First t >= 0 where the ray enters an axis-aligned box of half extents
/// `h`; 0 if it starts inside.
std::optional<double> HitBox(const Ray& r, const Vec3& h) {
  double t_min = 0.0, t_max = kHuge;
  const double o[3] = {r.o.x, r.o.y, r.o.z}, d[3] = {r.d.x, r.d.y, r.d.z}, half[3] = {h.x, h.y, h.z};
  for (int i = 0; i < 3; ++i) {
    if (std::abs(d[i]) < kEps) {
      if (std::abs(o[i]) > half[i]) return std::nullopt;
      continue;
    }
    double t1 = (-half[i] - o[i]) / d[i], t2 = (half[i] - o[i]) / d[i];
    if (t1 > t2) std::swap(t1, t2);
    t_min = std::max(t_min, t1);
    t_max = std::min(t_max, t2);
    if (t_max < t_min) return std::nullopt;
  }
  return t_min;
}

std::optional<double> HitSphere(const Ray& r, const Vec3& center, double radius) {
  const Vec3 o = r.o - center;
  const double c = Dot(o, o) - radius * radius;
  if (c <= 0.0) return 0.0;  // inside
  const double b = Dot(o, r.d);
  const double disc = b * b - c;
  if (disc < 0.0) return std::nullopt;
  const double t = -b - std::sqrt(disc);
  return t >= 0.0 ? std::optional<double>(t) : std::nullopt;
}

/// Finite cylinder along the local z axis: `radius`, z in [-half, half].
std::optional<double> HitCylinder(const Ray& r, double radius, double half) {
  const Vec3& o = r.o;
  const Vec3& d = r.d;
  if (o.x * o.x + o.y * o.y <= radius * radius && std::abs(o.z) <= half) return 0.0;
  std::optional<double> best;
  const auto consider = [&](double t) {
    if (t >= 0.0 && (!best || t < *best)) best = t;
  };
  const double a = d.x * d.x + d.y * d.y;
  if (a > kEps) {
    const double b = o.x * d.x + o.y * d.y;
    const double c = o.x * o.x + o.y * o.y - radius * radius;
    const double disc = b * b - a * c;
    if (disc >= 0.0) {
      const double t = (-b - std::sqrt(disc)) / a;
      if (std::abs(o.z + t * d.z) <= half) consider(t);
    }
  }
  if (std::abs(d.z) > kEps) {
    for (const double cap : {-half, half}) {
      const double t = (cap - o.z) / d.z;
      const double x = o.x + t * d.x, y = o.y + t * d.y;
      if (x * x + y * y <= radius * radius) consider(t);
    }
  }
  return best;
}

Ray LocalRay(const Shape& shape, double x, double y, double z_top) {
  const Quat inverse = shape.orientation.Conjugate();
  return Ray{inverse.Rotate(Vec3{x, y, z_top} - shape.position), inverse.Rotate(Vec3{0.0, 0.0, -1.0})};
}

}  // namespace

Vec3 Quat::Rotate(const Vec3& v) const {
  // v' = v + 2 w (q x v) + 2 q x (q x v), q = (x, y, z)
  const Vec3 q{x, y, z};
  const Vec3 t{2.0 * (q.y * v.z - q.z * v.y), 2.0 * (q.z * v.x - q.x * v.z), 2.0 * (q.x * v.y - q.y * v.x)};
  const Vec3 qt{q.y * t.z - q.z * t.y, q.z * t.x - q.x * t.z, q.x * t.y - q.y * t.x};
  return v + t * w + qt;
}

Quat Quat::FromRpy(double roll, double pitch, double yaw) {
  const double cr = std::cos(roll / 2), sr = std::sin(roll / 2);
  const double cp = std::cos(pitch / 2), sp = std::sin(pitch / 2);
  const double cy = std::cos(yaw / 2), sy = std::sin(yaw / 2);
  return Quat{cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
              cr * cp * sy - sr * sp * cy};
}

Bounds2 FootprintBounds(const Shape& shape) {
  Bounds2 b{kHuge, kHuge, -kHuge, -kHuge};
  const auto add = [&](const Vec3& p, double margin) {
    b.min_x = std::min(b.min_x, p.x - margin);
    b.min_y = std::min(b.min_y, p.y - margin);
    b.max_x = std::max(b.max_x, p.x + margin);
    b.max_y = std::max(b.max_y, p.y + margin);
  };
  const Shape& s = shape;
  switch (s.kind) {
    case Shape::Kind::kBox:
      for (const double dx : {-0.5, 0.5}) {
        for (const double dy : {-0.5, 0.5}) {
          for (const double dz : {-0.5, 0.5}) {
            add(s.position + s.orientation.Rotate(Vec3{dx * s.size.x, dy * s.size.y, dz * s.size.z}), 0.0);
          }
        }
      }
      break;
    case Shape::Kind::kSphere:
      add(s.position, s.size.x);
      break;
    case Shape::Kind::kCylinder:
    case Shape::Kind::kCapsule:
      for (const double dz : {-0.5, 0.5}) {
        add(s.position + s.orientation.Rotate(Vec3{0.0, 0.0, dz * s.size.z}), s.size.x);
      }
      break;
    case Shape::Kind::kPlane:
      if (s.size.x <= 0.0 || s.size.y <= 0.0) return Bounds2{-kHuge, -kHuge, kHuge, kHuge};
      for (const double dx : {-0.5, 0.5}) {
        for (const double dy : {-0.5, 0.5}) {
          add(s.position + s.orientation.Rotate(Vec3{dx * s.size.x, dy * s.size.y, 0.0}), 0.0);
        }
      }
      break;
  }
  return b;
}

std::optional<double> TopSurface(const Shape& shape, double x, double y, double z_top) {
  const Ray r = LocalRay(shape, x, y, z_top);
  std::optional<double> t;
  switch (shape.kind) {
    case Shape::Kind::kBox:
      t = HitBox(r, shape.size * 0.5);
      break;
    case Shape::Kind::kSphere:
      t = HitSphere(r, Vec3{}, shape.size.x);
      break;
    case Shape::Kind::kCylinder:
      t = HitCylinder(r, shape.size.x, 0.5 * shape.size.z);
      break;
    case Shape::Kind::kCapsule: {
      const double half = 0.5 * shape.size.z;
      for (const auto hit : {HitCylinder(r, shape.size.x, half), HitSphere(r, Vec3{0.0, 0.0, -half}, shape.size.x),
                             HitSphere(r, Vec3{0.0, 0.0, half}, shape.size.x)}) {
        if (hit && (!t || *hit < *t)) t = hit;
      }
      break;
    }
    case Shape::Kind::kPlane: {
      if (std::abs(r.d.z) < kEps) return std::nullopt;
      const double hit = -r.o.z / r.d.z;
      if (hit < 0.0) return std::nullopt;
      const double px = r.o.x + hit * r.d.x, py = r.o.y + hit * r.d.y;
      const bool finite = shape.size.x > 0.0 && shape.size.y > 0.0;
      if (finite && (std::abs(px) > 0.5 * shape.size.x || std::abs(py) > 0.5 * shape.size.y)) return std::nullopt;
      t = hit;
      break;
    }
  }
  if (!t) return std::nullopt;
  return z_top - *t;  // the ray points straight down with unit speed
}

std::size_t FillStatic(HeightMap& map, const std::vector<Shape>& shapes, double z_top) {
  std::vector<Bounds2> bounds;
  bounds.reserve(shapes.size());
  for (const Shape& s : shapes) bounds.push_back(FootprintBounds(s));
  const auto cells = map.UnknownStaticCells();
  for (const CellIndex c : cells) {
    const Point2 p = map.geometry().Center(c);
    double top = kNoSurface;
    for (std::size_t i = 0; i < shapes.size(); ++i) {
      const Bounds2& b = bounds[i];
      if (p.x < b.min_x || p.x > b.max_x || p.y < b.min_y || p.y > b.max_y) continue;
      if (const auto z = TopSurface(shapes[i], p.x, p.y, z_top)) top = std::max(top, *z);
    }
    map.SetStatic(c, static_cast<float>(top));
  }
  return cells.size();
}

void DrawMovable(HeightMap& map, const std::vector<Shape>& shapes, double z_top) {
  map.ClearMovable();
  const GridGeometry& g = map.geometry();
  for (const Shape& s : shapes) {
    const Bounds2 b = FootprintBounds(s);
    const CellIndex lo = g.CellAtUnchecked({b.min_x, b.min_y});
    const CellIndex hi = g.CellAtUnchecked({b.max_x, b.max_y});
    for (int y = std::max(0, lo.y); y <= std::min(g.size_y() - 1, hi.y); ++y) {
      for (int x = std::max(0, lo.x); x <= std::min(g.size_x() - 1, hi.x); ++x) {
        const Point2 p = g.Center({x, y});
        if (const auto z = TopSurface(s, p.x, p.y, z_top)) map.SetMovable({x, y}, static_cast<float>(*z));
      }
    }
  }
}

}  // namespace spot_sim::planning
