#pragma once

#include <array>
#include <optional>

#include "spot_sim_core/geometry.hpp"
#include "spot_sim_core/leg_kinematics.hpp"

namespace spot_sim {

/// Body posture relative to nominal standing, expressed in the footprint
/// frame (gravity aligned, heading of the feet): what MobilityParams'
/// body_control asks for.
struct BodyPosture {
  double height_offset = 0.0;  ///< [m] relative to the nominal stand height
  double roll = 0.0, pitch = 0.0, yaw = 0.0;  ///< [rad], rotation Rz(yaw) Ry(pitch) Rx(roll)

  /// Requests beyond these are saturated, as on the real robot (the driver
  /// documents stand heights of -0.15 m crouching to 0.15 m on tiptoes).
  /// Combinations the legs cannot reach are saturated further by the base.
  static BodyPosture Min() { return {-0.15, -0.4, -0.4, -0.5}; }
  static BodyPosture Max() { return {0.15, 0.4, 0.4, 0.5}; }
  BodyPosture Clamped() const;

  BodyPosture Scaled(double factor) const {
    return {factor * height_offset, factor * roll, factor * pitch, factor * yaw};
  }
  bool operator==(const BodyPosture& o) const {
    return height_offset == o.height_offset && roll == o.roll && pitch == o.pitch && yaw == o.yaw;
  }
};

/// Standing on flat ground: where the body is and how the legs reach the
/// feet. The ground frame has its origin on the ground below the nominal body
/// position and the footprint's heading; the neutral foot points are below the
/// hips at nominal stance.
struct Stance {
  using FootOffsets = std::array<Vec3, 4>;

  /// Body origin in the ground frame.
  static Pose BodyInGround(double stand_height, const BodyPosture& posture);

  /// Joint positions for the body at `posture` with each foot `foot_offsets`
  /// away from its neutral point (ground frame). Empty if a foot is out of
  /// reach.
  static std::optional<std::array<LegJoints, 4>> Solve(double stand_height, const BodyPosture& posture,
                                                       const FootOffsets& foot_offsets = {});
};

}  // namespace spot_sim
