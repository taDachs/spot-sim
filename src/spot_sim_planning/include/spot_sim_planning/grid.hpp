#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

namespace spot_sim::planning {

/// A point in the map frame (gravity-aligned, z up; the planner uses the
/// world/odom plane) [m].
struct Point2 {
  double x = 0.0, y = 0.0;

  Point2 operator+(const Point2& o) const { return {x + o.x, y + o.y}; }
  Point2 operator-(const Point2& o) const { return {x - o.x, y - o.y}; }
  Point2 operator*(double s) const { return {x * s, y * s}; }
  double Norm() const { return std::hypot(x, y); }
};

struct CellIndex {
  int x = 0, y = 0;
  bool operator==(const CellIndex& o) const { return x == o.x && y == o.y; }
  bool operator!=(const CellIndex& o) const { return !(*this == o); }
};

/// Placement of a regular grid in the map frame: cell (i, j) covers
/// [origin.x + i * resolution, origin.x + (i + 1) * resolution) in x, and
/// likewise in y. Cells are stored row by row (x fastest).
class GridGeometry {
 public:
  GridGeometry() = default;
  GridGeometry(double resolution, int size_x, int size_y, Point2 origin)
      : resolution_(resolution), size_x_(size_x), size_y_(size_y), origin_(origin) {}

  double resolution() const { return resolution_; }
  int size_x() const { return size_x_; }
  int size_y() const { return size_y_; }
  std::size_t cell_count() const { return static_cast<std::size_t>(size_x_) * static_cast<std::size_t>(size_y_); }
  /// Map coordinates of the outer corner of cell (0, 0).
  Point2 origin() const { return origin_; }
  /// Map coordinates of the window's centre.
  Point2 center() const { return {origin_.x + 0.5 * size_x_ * resolution_, origin_.y + 0.5 * size_y_ * resolution_}; }

  bool Contains(CellIndex c) const { return c.x >= 0 && c.y >= 0 && c.x < size_x_ && c.y < size_y_; }
  bool Contains(Point2 p) const { return Contains(CellAtUnchecked(p)); }

  /// The cell containing `p` (possibly outside the grid).
  CellIndex CellAtUnchecked(Point2 p) const {
    return {static_cast<int>(std::floor((p.x - origin_.x) / resolution_)),
            static_cast<int>(std::floor((p.y - origin_.y) / resolution_))};
  }
  std::optional<CellIndex> CellAt(Point2 p) const {
    const CellIndex c = CellAtUnchecked(p);
    return Contains(c) ? std::optional<CellIndex>(c) : std::nullopt;
  }

  Point2 Center(CellIndex c) const {
    return {origin_.x + (c.x + 0.5) * resolution_, origin_.y + (c.y + 0.5) * resolution_};
  }

  std::size_t Index(CellIndex c) const {
    return static_cast<std::size_t>(c.y) * static_cast<std::size_t>(size_x_) + static_cast<std::size_t>(c.x);
  }
  CellIndex FromIndex(std::size_t i) const {
    return {static_cast<int>(i % static_cast<std::size_t>(size_x_)), static_cast<int>(i / static_cast<std::size_t>(size_x_))};
  }

  bool operator==(const GridGeometry& o) const {
    return resolution_ == o.resolution_ && size_x_ == o.size_x_ && size_y_ == o.size_y_ && origin_.x == o.origin_.x &&
           origin_.y == o.origin_.y;
  }

 private:
  double resolution_ = 1.0;
  int size_x_ = 0, size_y_ = 0;
  Point2 origin_;
};

/// Values of type T on a GridGeometry.
template <typename T>
class Grid {
 public:
  Grid() = default;
  Grid(const GridGeometry& geometry, T value) : geometry_(geometry), data_(geometry.cell_count(), value) {}

  const GridGeometry& geometry() const { return geometry_; }
  std::size_t size() const { return data_.size(); }

  T& operator[](std::size_t i) { return data_[i]; }
  const T& operator[](std::size_t i) const { return data_[i]; }
  T& at(CellIndex c) { return data_[geometry_.Index(c)]; }
  const T& at(CellIndex c) const { return data_[geometry_.Index(c)]; }

  void Fill(T value) { std::fill(data_.begin(), data_.end(), value); }
  std::vector<T>& data() { return data_; }
  const std::vector<T>& data() const { return data_; }

 private:
  GridGeometry geometry_;
  std::vector<T> data_;
};

/// Bilinear interpolation of `grid` (values at cell centres) at `p`.
/// Empty outside the grid's cell centres or when a corner value is not
/// finite (unknown / unreachable).
template <typename T>
std::optional<double> Interpolate(const Grid<T>& grid, Point2 p) {
  const GridGeometry& g = grid.geometry();
  const double fx = (p.x - g.origin().x) / g.resolution() - 0.5;
  const double fy = (p.y - g.origin().y) / g.resolution() - 0.5;
  const int x0 = static_cast<int>(std::floor(fx)), y0 = static_cast<int>(std::floor(fy));
  if (x0 < 0 || y0 < 0 || x0 + 1 >= g.size_x() || y0 + 1 >= g.size_y()) {
    return std::nullopt;
  }
  const double tx = fx - x0, ty = fy - y0;
  const double v00 = grid.at({x0, y0}), v10 = grid.at({x0 + 1, y0});
  const double v01 = grid.at({x0, y0 + 1}), v11 = grid.at({x0 + 1, y0 + 1});
  if (!std::isfinite(v00) || !std::isfinite(v10) || !std::isfinite(v01) || !std::isfinite(v11)) {
    return std::nullopt;
  }
  return (1 - ty) * ((1 - tx) * v00 + tx * v10) + ty * ((1 - tx) * v01 + tx * v11);
}

}  // namespace spot_sim::planning
