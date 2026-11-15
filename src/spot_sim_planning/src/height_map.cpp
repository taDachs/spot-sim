#include "spot_sim_planning/height_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace spot_sim::planning {

namespace {
constexpr float kUnknown = std::numeric_limits<float>::quiet_NaN();

/// Copies the values of `from` that `to`'s geometry also covers (same lattice).
void CopyOverlap(const Grid<float>& from, Grid<float>& to) {
  const GridGeometry& f = from.geometry();
  const GridGeometry& t = to.geometry();
  const int dx = static_cast<int>(std::lround((t.origin().x - f.origin().x) / f.resolution()));
  const int dy = static_cast<int>(std::lround((t.origin().y - f.origin().y) / f.resolution()));
  for (int y = 0; y < t.size_y(); ++y) {
    const int fy = y + dy;
    if (fy < 0 || fy >= f.size_y()) continue;
    for (int x = 0; x < t.size_x(); ++x) {
      const int fx = x + dx;
      if (fx < 0 || fx >= f.size_x()) continue;
      to.at({x, y}) = from.at({fx, fy});
    }
  }
}
}  // namespace

HeightMap::HeightMap(int size, double resolution) {
  const GridGeometry centred(resolution, size, size, SnappedOrigin(Point2{}, size, resolution));
  static_ = Grid<float>(centred, kUnknown);
  movable_ = Grid<float>(centred, kUnknown);
}

Point2 HeightMap::SnappedOrigin(Point2 center, int size, double resolution) {
  // The cell containing `center` becomes cell (size / 2, size / 2).
  const double r = resolution;
  return Point2{(std::floor(center.x / r) - size / 2) * r, (std::floor(center.y / r) - size / 2) * r};
}

bool HeightMap::Recenter(Point2 center) {
  const GridGeometry& old = static_.geometry();
  const Point2 origin = SnappedOrigin(center, old.size_x(), old.resolution());
  if (std::abs(origin.x - old.origin().x) < 0.5 * old.resolution() &&
      std::abs(origin.y - old.origin().y) < 0.5 * old.resolution()) {
    return false;
  }
  const GridGeometry moved(old.resolution(), old.size_x(), old.size_y(), origin);
  Grid<float> new_static(moved, kUnknown), new_movable(moved, kUnknown);
  CopyOverlap(static_, new_static);
  CopyOverlap(movable_, new_movable);
  static_ = std::move(new_static);
  movable_ = std::move(new_movable);
  return true;
}

bool HeightMap::RecenterIfNeeded(Point2 center, double slack) {
  const Point2 c = geometry().center();
  if (std::abs(center.x - c.x) <= slack && std::abs(center.y - c.y) <= slack) {
    return false;
  }
  return Recenter(center);
}

std::vector<CellIndex> HeightMap::UnknownStaticCells() const {
  std::vector<CellIndex> cells;
  for (std::size_t i = 0; i < static_.size(); ++i) {
    if (std::isnan(static_[i])) cells.push_back(static_.geometry().FromIndex(i));
  }
  return cells;
}

void HeightMap::InvalidateStatic() { static_.Fill(kUnknown); }

void HeightMap::ClearMovable() { movable_.Fill(kUnknown); }

void HeightMap::SetMovable(CellIndex c, float height) {
  float& cell = movable_.at(c);
  cell = std::isnan(cell) ? height : std::max(cell, height);
}

float HeightMap::Height(CellIndex c) const {
  const float s = static_.at(c), m = movable_.at(c);
  if (std::isnan(s)) return m;
  if (std::isnan(m)) return s;
  return std::max(s, m);
}

Grid<float> HeightMap::Heights() const {
  Grid<float> heights(geometry(), kUnknown);
  for (std::size_t i = 0; i < heights.size(); ++i) {
    heights[i] = Height(geometry().FromIndex(i));
  }
  return heights;
}

std::optional<double> HeightMap::HeightAt(Point2 p) const {
  const GridGeometry& g = geometry();
  const double fx = (p.x - g.origin().x) / g.resolution() - 0.5;
  const double fy = (p.y - g.origin().y) / g.resolution() - 0.5;
  const int x0 = static_cast<int>(std::floor(fx)), y0 = static_cast<int>(std::floor(fy));
  if (x0 < 0 || y0 < 0 || x0 + 1 >= g.size_x() || y0 + 1 >= g.size_y()) {
    return std::nullopt;
  }
  const double tx = fx - x0, ty = fy - y0;
  const double v00 = Height({x0, y0}), v10 = Height({x0 + 1, y0});
  const double v01 = Height({x0, y0 + 1}), v11 = Height({x0 + 1, y0 + 1});
  if (std::isnan(v00) || std::isnan(v10) || std::isnan(v01) || std::isnan(v11)) {
    return std::nullopt;
  }
  return (1 - ty) * ((1 - tx) * v00 + tx * v10) + ty * ((1 - tx) * v01 + tx * v11);
}

}  // namespace spot_sim::planning
