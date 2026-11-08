#pragma once

#include <array>
#include <cstddef>
#include <optional>

#include "spot_sim_core/geometry.hpp"

namespace spot_sim {

/// Joint positions of one leg.
struct LegJoints {
  double hip_x = 0.0;
  double hip_y = 0.0;
  double knee = 0.0;
};

/// A simple trot for animating a kinematic base: diagonal leg pairs
/// (fl + hr, fr + hl) alternate between stance and swing. Stance feet move
/// backwards under the body at the body's velocity, so they stay put on the
/// ground; swing feet move forward on an arc. Purely visual: the body motion
/// does not depend on it.
///
/// Legs are indexed fl, fr, hl, hr (Spot API order).
class TrotGait {
 public:
  struct Params {
    double period = 0.5;        ///< full gait cycle [s]
    double step_height = 0.08;  ///< swing foot lift [m]
    double blend_time = 0.15;   ///< fade in / out when starting / stopping [s]
  };

  TrotGait() : TrotGait(Params{}) {}
  explicit TrotGait(Params params);

  /// Advances the gait by `dt` while the body moves at the given planar
  /// velocity (body frame, [m/s] and [rad/s]).
  void Step(double dt, double v_x, double v_y, double v_rot);

  /// Each foot's displacement from its neutral stance point below the hip
  /// (body frame). All zero when the robot stands still.
  std::array<Vec3, 4> FootOffsets() const;

  /// True for legs currently carrying the body (not swinging).
  bool InStance(std::size_t leg) const;

  /// True while the gait is (fading) active.
  bool Active() const { return weight_ > 0.0; }

  /// Joint positions placing `leg`'s foot `offset` away from its neutral
  /// stance point, for a body origin `body_height` above flat ground.
  /// Empty if out of reach.
  static std::optional<LegJoints> Solve(std::size_t leg, const Vec3& offset, double body_height);

 private:
  double LegPhase(std::size_t leg) const;

  Params params_;
  double phase_ = 0.0;   ///< [0, 1)
  double weight_ = 0.0;  ///< 0 = standing still, 1 = full gait
  double v_x_ = 0.0, v_y_ = 0.0, v_rot_ = 0.0;
};

}  // namespace spot_sim
