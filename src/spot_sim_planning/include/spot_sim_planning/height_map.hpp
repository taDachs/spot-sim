#pragma once

#include <optional>
#include <vector>

#include "spot_sim_planning/grid.hpp"

namespace spot_sim::planning {

/// Robot-centred 2.5D map: the height [m] of the top surface in each cell of
/// a square window in the map frame, which rolls along with the robot.
///
/// Two layers, filled by whoever feeds the map (the Gazebo plugin casts rays
/// onto the world's collision geometry; depth cameras could feed it later):
///
/// - static: things that never move. Filled once per cell; cells that come
///   into view when the window rolls start unknown (UnknownStaticCells()).
/// - movable: things that may move, re-drawn every planning cycle
///   (ClearMovable(), then SetMovable()). Moving obstacles are simply part
///   of the map; the planner reacts by replanning.
///
/// The map height of a cell is the higher of the two. Unknown cells are NaN.
class HeightMap {
 public:
  /// A `size` x `size` cell window with cells of `resolution` [m], centred on
  /// the map origin until the first Recenter().
  HeightMap(int size, double resolution);

  const GridGeometry& geometry() const { return static_.geometry(); }

  /// Moves the window so that `center` lies in its central cell. The window
  /// snaps to a fixed lattice of cells (multiples of the resolution), so cells
  /// keep their place in the world: those still covered keep their values,
  /// newly covered ones are unknown. Returns whether the window moved.
  bool Recenter(Point2 center);
  /// Recenter() only once `center` is more than `slack` [m] (per axis) from
  /// the window's centre, so the window doesn't move every step.
  bool RecenterIfNeeded(Point2 center, double slack);

  float Static(CellIndex c) const { return static_.at(c); }
  void SetStatic(CellIndex c, float height) { static_.at(c) = height; }
  /// Cells whose static height is unknown (to be filled by the feeder).
  std::vector<CellIndex> UnknownStaticCells() const;
  /// Forgets the static layer (e.g. after a static model was added).
  void InvalidateStatic();

  float Movable(CellIndex c) const { return movable_.at(c); }
  void ClearMovable();
  /// Raises the movable layer at `c` to at least `height`.
  void SetMovable(CellIndex c, float height);

  /// Height of the cell's top surface: the higher layer; NaN if both unknown.
  float Height(CellIndex c) const;
  /// All cells' heights (for the traversability computation).
  Grid<float> Heights() const;
  /// Height at `p`, bilinear between cell centres; empty outside the window
  /// or next to unknown cells.
  std::optional<double> HeightAt(Point2 p) const;

 private:
  /// Window origin (corner of cell (0, 0)) for a window centred on `center`.
  static Point2 SnappedOrigin(Point2 center, int size, double resolution);

  Grid<float> static_;
  Grid<float> movable_;
};

}  // namespace spot_sim::planning
