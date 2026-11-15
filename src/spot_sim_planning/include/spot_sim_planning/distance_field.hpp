#pragma once

#include <cstdint>

#include "spot_sim_planning/grid.hpp"

namespace spot_sim::planning {

/// Signed Euclidean distance [m] between cell centres: in free cells the
/// distance to the nearest obstacle cell, in obstacle cells minus the distance
/// to the nearest free cell. +infinity everywhere if there is no obstacle
/// (-infinity if everything is an obstacle).
///
/// Distances are between cell centres, so the true distance to an
/// obstacle's edge can be up to half a cell less: callers checking a circle
/// of radius r should require distance >= r + resolution / 2.
Grid<float> SignedDistance(const Grid<uint8_t>& obstacle);

}  // namespace spot_sim::planning
