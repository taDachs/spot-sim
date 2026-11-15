#include "spot_sim_planning/traversability.hpp"

#include <algorithm>
#include <cmath>

namespace spot_sim::planning {

namespace {

struct PlaneFit {
  double slope = 0.0;     ///< [rad]
  double residual = 0.0;  ///< RMS [m]
  bool ok = false;
};

/// Least-squares plane z = a x + b y + c through the known heights in the
/// (2k+1)^2 window around `c` (cell units for x and y).
PlaneFit FitPlane(const Grid<float>& heights, CellIndex center, int k) {
  const GridGeometry& g = heights.geometry();
  double sxx = 0, sxy = 0, syy = 0, sx = 0, sy = 0, sz = 0, sxz = 0, syz = 0, szz = 0;
  int n = 0;
  for (int dy = -k; dy <= k; ++dy) {
    for (int dx = -k; dx <= k; ++dx) {
      const CellIndex c{center.x + dx, center.y + dy};
      if (!g.Contains(c)) continue;
      const double z = heights.at(c);
      if (std::isnan(z)) continue;
      const double x = dx, y = dy;
      sxx += x * x, sxy += x * y, syy += y * y, sx += x, sy += y, sz += z, sxz += x * z, syz += y * z, szz += z * z;
      ++n;
    }
  }
  PlaneFit fit;
  if (n < 6) return fit;
  // Normal equations with centred coordinates.
  const double mx = sx / n, my = sy / n, mz = sz / n;
  const double cxx = sxx - n * mx * mx, cxy = sxy - n * mx * my, cyy = syy - n * my * my;
  const double cxz = sxz - n * mx * mz, cyz = syz - n * my * mz, czz = szz - n * mz * mz;
  const double det = cxx * cyy - cxy * cxy;
  if (std::abs(det) < 1e-9) return fit;
  const double a = (cxz * cyy - cyz * cxy) / det;  // per cell
  const double b = (cyz * cxx - cxz * cxy) / det;
  // Residual sum of squares = czz - (a cxz + b cyz).
  const double rss = std::max(0.0, czz - (a * cxz + b * cyz));
  const double r = g.resolution();
  fit.slope = std::atan(std::hypot(a, b) / r);
  fit.residual = std::sqrt(rss / n);
  fit.ok = true;
  return fit;
}

/// Height range (max - min) of the known cells in the 3x3 neighbourhood.
double StepHeight(const Grid<float>& heights, CellIndex center) {
  const GridGeometry& g = heights.geometry();
  double lo = INFINITY, hi = -INFINITY;
  for (int dy = -1; dy <= 1; ++dy) {
    for (int dx = -1; dx <= 1; ++dx) {
      const CellIndex c{center.x + dx, center.y + dy};
      if (!g.Contains(c)) continue;
      const double z = heights.at(c);
      if (std::isnan(z)) continue;
      lo = std::min(lo, z);
      hi = std::max(hi, z);
    }
  }
  return hi >= lo ? hi - lo : 0.0;
}

}  // namespace

Traversability ComputeTraversability(const Grid<float>& heights, const TraversabilityParams& params) {
  const GridGeometry& g = heights.geometry();
  Traversability t{Grid<uint8_t>(g, 0), Grid<float>(g, 1.0f), Grid<float>(g, 0.0f), Grid<float>(g, 0.0f)};
  const int k = std::max(1, static_cast<int>(std::lround(params.slope_radius / g.resolution())));
  for (std::size_t i = 0; i < heights.size(); ++i) {
    const CellIndex c = g.FromIndex(i);
    if (std::isnan(heights[i])) {
      t.obstacle[i] = params.unknown_is_obstacle ? 1 : 0;
      continue;
    }
    const double step = StepHeight(heights, c);
    const PlaneFit plane = FitPlane(heights, c, k);
    // A slope only counts where the surface really is a plane; at step edges
    // and on rough ground the step rule decides.
    const double slope = plane.ok && plane.residual <= params.max_plane_residual ? plane.slope : 0.0;
    t.step[i] = static_cast<float>(step);
    t.slope[i] = static_cast<float>(slope);
    if (step > params.max_step || slope > params.max_slope) {
      t.obstacle[i] = 1;
      continue;
    }
    t.cost[i] = static_cast<float>(1.0 + params.slope_cost * slope / params.max_slope +
                                   params.step_cost * step / params.max_step);
  }
  return t;
}

}  // namespace spot_sim::planning
