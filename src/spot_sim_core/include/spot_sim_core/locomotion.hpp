#pragma once

#include <cstdint>

namespace spot_sim {

/// What the robot's "firmware" (the Spot API side) asks the locomotion base
/// to do. Plain data, exchanged with the base plugin through Gazebo
/// components; any base (kinematic, RL policy, ...) consumes the same struct.
enum class LocomotionMode : uint8_t {
  kSit,       ///< fold legs, rest on the ground
  kStand,     ///< stand still
  kVelocity,  ///< walk at the commanded body velocity (slice 4)
  kStop,      ///< stop walking, keep the current posture
};

struct LocomotionCommand {
  bool motors_on = false;
  LocomotionMode mode = LocomotionMode::kSit;
  /// Increases with every new command; the base echoes it once reached.
  uint32_t sequence = 0;

  /// Body velocity target in the (gravity-aligned) body frame [m/s, rad/s]
  /// (kVelocity), already clamped to the velocity limits.
  double v_x = 0.0, v_y = 0.0, v_rot = 0.0;
  /// How fast the base may change the velocity [m/s^2, rad/s^2].
  double max_accel_x = 2.0, max_accel_y = 2.0, max_accel_rot = 2.0;
  /// Body posture relative to nominal standing (slice 5).
  double body_height_offset = 0.0;
  double body_roll = 0.0, body_pitch = 0.0, body_yaw = 0.0;

  bool operator==(const LocomotionCommand& o) const {
    return motors_on == o.motors_on && mode == o.mode && sequence == o.sequence && v_x == o.v_x && v_y == o.v_y &&
           v_rot == o.v_rot && max_accel_x == o.max_accel_x && max_accel_y == o.max_accel_y &&
           max_accel_rot == o.max_accel_rot && body_height_offset == o.body_height_offset && body_roll == o.body_roll &&
           body_pitch == o.body_pitch && body_yaw == o.body_yaw;
  }
  bool operator!=(const LocomotionCommand& o) const { return !(*this == o); }
};

enum class LocomotionState : uint8_t {
  kSitting,
  kStanding,
  kTransitioning,  ///< standing up, sitting down or collapsing
  kMoving,        ///< standing and walking
};

/// What the base reports back. Everything else (pose, joints, contacts) the
/// bridge reads from Gazebo itself.
struct LocomotionStatus {
  LocomotionState state = LocomotionState::kSitting;
  /// Sequence of the last command whose target posture has been reached.
  uint32_t reached_sequence = 0;

  bool operator==(const LocomotionStatus& o) const {
    return state == o.state && reached_sequence == o.reached_sequence;
  }
  bool operator!=(const LocomotionStatus& o) const { return !(*this == o); }
};

}  // namespace spot_sim
