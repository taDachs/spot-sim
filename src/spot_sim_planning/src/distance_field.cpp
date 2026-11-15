#include "spot_sim_planning/distance_field.hpp"

#include <cmath>
#include <limits>
#include <vector>

namespace spot_sim::planning {

namespace {

constexpr double kInf = 1e20;

/// Felzenszwalb & Huttenlocher 1D squared distance transform of `f` (in
/// place), with `v` / `z` as scratch space.
void Transform1d(std::vector<double>& f, std::vector<int>& v, std::vector<double>& z, std::vector<double>& out) {
  const int n = static_cast<int>(f.size());
  int k = 0;
  v[0] = 0;
  z[0] = -kInf;
  z[1] = kInf;
  const auto intersection = [&](int q, int p) {
    return ((f[q] + double(q) * q) - (f[p] + double(p) * p)) / (2.0 * q - 2.0 * p);
  };
  for (int q = 1; q < n; ++q) {
    double s = intersection(q, v[k]);
    while (s <= z[k]) {  // z[0] = -inf ends the loop at k = 0
      --k;
      s = intersection(q, v[k]);
    }
    ++k;
    v[k] = q;
    z[k] = s;
    z[k + 1] = kInf;
  }
  k = 0;
  for (int q = 0; q < n; ++q) {
    while (z[k + 1] < q) ++k;
    const double d = q - v[k];
    out[q] = d * d + f[v[k]];
  }
  f.swap(out);
}

/// Squared distance [cells^2] from every cell to the nearest cell where
/// `target` is true (kInf if none).
std::vector<double> SquaredDistance(const Grid<uint8_t>& mask, bool target) {
  const GridGeometry& g = mask.geometry();
  const int nx = g.size_x(), ny = g.size_y();
  std::vector<double> d(g.cell_count());
  for (std::size_t i = 0; i < d.size(); ++i) {
    d[i] = (mask[i] != 0) == target ? 0.0 : kInf;
  }
  const int n = std::max(nx, ny);
  std::vector<double> f, out(n);
  std::vector<int> v(n);
  std::vector<double> z(n + 1);
  // Columns, then rows.
  f.resize(ny);
  out.resize(ny);
  for (int x = 0; x < nx; ++x) {
    for (int y = 0; y < ny; ++y) f[y] = d[static_cast<std::size_t>(y) * nx + x];
    Transform1d(f, v, z, out);
    for (int y = 0; y < ny; ++y) d[static_cast<std::size_t>(y) * nx + x] = f[y];
  }
  f.resize(nx);
  out.resize(nx);
  for (int y = 0; y < ny; ++y) {
    for (int x = 0; x < nx; ++x) f[x] = d[static_cast<std::size_t>(y) * nx + x];
    Transform1d(f, v, z, out);
    for (int x = 0; x < nx; ++x) d[static_cast<std::size_t>(y) * nx + x] = f[x];
  }
  return d;
}

}  // namespace

Grid<float> SignedDistance(const Grid<uint8_t>& obstacle) {
  const GridGeometry& g = obstacle.geometry();
  const auto to_obstacle = SquaredDistance(obstacle, true);
  const auto to_free = SquaredDistance(obstacle, false);
  Grid<float> out(g, 0.0f);
  const double r = g.resolution();
  constexpr float kFloatInf = std::numeric_limits<float>::infinity();
  for (std::size_t i = 0; i < out.size(); ++i) {
    if (obstacle[i] == 0) {
      out[i] = to_obstacle[i] >= kInf ? kFloatInf : static_cast<float>(std::sqrt(to_obstacle[i]) * r);
    } else {
      out[i] = to_free[i] >= kInf ? -kFloatInf : -static_cast<float>(std::sqrt(to_free[i]) * r);
    }
  }
  return out;
}

}  // namespace spot_sim::planning
