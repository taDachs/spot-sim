#include "spot_sim_planning/cost_to_go.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

namespace spot_sim::planning {

namespace {
constexpr float kInf = std::numeric_limits<float>::infinity();
}  // namespace

Grid<float> ComputeCostToGo(const Traversability& traversability, const Grid<float>& distance, Point2 goal,
                            const CostToGoParams& params) {
  const GridGeometry& g = traversability.obstacle.geometry();
  const double r = g.resolution();

  // Per-cell cost of walking through it; +inf where impassable.
  std::vector<float> cell_cost(g.cell_count(), kInf);
  for (std::size_t i = 0; i < cell_cost.size(); ++i) {
    if (traversability.obstacle[i] != 0 || distance[i] < params.inflation_radius) continue;
    double cost = traversability.cost[i];
    if (params.proximity_radius > params.inflation_radius && distance[i] < params.proximity_radius) {
      const double closeness = (params.proximity_radius - distance[i]) /
                               (params.proximity_radius - params.inflation_radius);
      cost *= 1.0 + params.proximity_weight * closeness;
    }
    cell_cost[i] = static_cast<float>(cost);
  }

  Grid<float> cost_to_go(g, kInf);
  using Entry = std::pair<float, std::size_t>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
  const auto seed = [&](CellIndex c, double cost) {
    const std::size_t i = g.Index(c);
    if (!std::isfinite(cell_cost[i]) || cost >= cost_to_go[i]) return;
    cost_to_go[i] = static_cast<float>(cost);
    open.push({cost_to_go[i], i});
  };
  if (const auto goal_cell = g.CellAt(goal)) {
    seed(*goal_cell, 0.0);
  } else {
    for (int x = 0; x < g.size_x(); ++x) {
      for (const int y : {0, g.size_y() - 1}) seed({x, y}, (g.Center({x, y}) - goal).Norm());
    }
    for (int y = 1; y + 1 < g.size_y(); ++y) {
      for (const int x : {0, g.size_x() - 1}) seed({x, y}, (g.Center({x, y}) - goal).Norm());
    }
  }

  constexpr int kDx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  constexpr int kDy[8] = {0, 0, 1, -1, 1, -1, 1, -1};
  const double diagonal = std::sqrt(2.0);
  while (!open.empty()) {
    const auto [cost, i] = open.top();
    open.pop();
    if (cost > cost_to_go[i]) continue;  // stale entry
    const CellIndex c = g.FromIndex(i);
    for (int k = 0; k < 8; ++k) {
      const CellIndex n{c.x + kDx[k], c.y + kDy[k]};
      if (!g.Contains(n)) continue;
      const std::size_t j = g.Index(n);
      if (!std::isfinite(cell_cost[j])) continue;
      if (k >= 4) {
        // No corner cutting past impassable cells.
        if (!std::isfinite(cell_cost[g.Index({c.x + kDx[k], c.y})]) ||
            !std::isfinite(cell_cost[g.Index({c.x, c.y + kDy[k]})])) {
          continue;
        }
      }
      const double step = (k < 4 ? 1.0 : diagonal) * r * 0.5 * (cell_cost[i] + cell_cost[j]);
      const float next = static_cast<float>(cost + step);
      if (next < cost_to_go[j]) {
        cost_to_go[j] = next;
        open.push({next, j});
      }
    }
  }
  return cost_to_go;
}

Grid<float> ExtendCostToGo(const Grid<float>& cost_to_go, double radius, double weight) {
  const GridGeometry& g = cost_to_go.geometry();
  const double r = g.resolution();
  Grid<float> out = cost_to_go;
  // Distance travelled into the unreachable region, per cell.
  std::vector<float> travelled(g.cell_count(), kInf);
  using Entry = std::pair<float, std::size_t>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
  for (std::size_t i = 0; i < out.size(); ++i) {
    if (std::isfinite(out[i])) {
      travelled[i] = 0.0f;
      open.push({out[i], i});
    }
  }
  constexpr int kDx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  constexpr int kDy[8] = {0, 0, 1, -1, 1, -1, 1, -1};
  const double diagonal = std::sqrt(2.0);
  while (!open.empty()) {
    const auto [cost, i] = open.top();
    open.pop();
    if (cost > out[i]) continue;
    const CellIndex c = g.FromIndex(i);
    for (int k = 0; k < 8; ++k) {
      const CellIndex n{c.x + kDx[k], c.y + kDy[k]};
      if (!g.Contains(n)) continue;
      const std::size_t j = g.Index(n);
      if (std::isfinite(cost_to_go[j])) continue;  // only fill unreachable cells
      const double length = (k < 4 ? 1.0 : diagonal) * r;
      const double distance = travelled[i] + length;
      if (distance > radius) continue;
      const float next = static_cast<float>(cost + weight * length);
      if (next < out[j]) {
        out[j] = next;
        travelled[j] = static_cast<float>(distance);
        open.push({next, j});
      }
    }
  }
  return out;
}

}  // namespace spot_sim::planning
