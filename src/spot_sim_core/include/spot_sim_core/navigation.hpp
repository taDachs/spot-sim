#pragma once

#include <cstdint>

namespace spot_sim {

/// What the robot's "firmware" asks the local planner for while walking.
/// Plain data, exchanged with the planner plugin through a Gazebo component
/// like LocomotionCommand; any planner consumes the same struct.
///
/// The planner only produces the walking velocity: it rewrites v_x, v_y,
/// v_rot of the LocomotionCommand while its mode is kVelocity. Power, sitting,
/// standing and posture go from the firmware to the base untouched.
struct NavigationRequest {
  enum class Kind : uint8_t {
    kNone,      ///< not walking: the planner leaves the command alone
    kVelocity,  ///< walk at LocomotionCommand::v_* (with obstacle avoidance if `avoid`)
    kGoal,      ///< walk to `goal_*`, around obstacles
  };
  Kind kind = Kind::kNone;
  /// Changes with every new goal.
  uint32_t sequence = 0;

  /// kVelocity: avoid obstacles (MobilityParams obstacle_params
  /// disable_vision_body_obstacle_avoidance = false). Without it the
  /// velocity passes through unchanged.
  bool avoid = true;
  /// Clearance to keep from obstacles [m] (obstacle_avoidance_padding).
  double padding = 0.1;
  /// Velocity limits [m/s, m/s, rad/s].
  double max_vx = 1.6, max_vy = 0.6, max_wz = 1.2;

  /// kGoal: the goal pose in odom, and when it counts as reached.
  double goal_x = 0.0, goal_y = 0.0, goal_yaw = 0.0;
  double position_tolerance = 0.1;  ///< [m]
  double yaw_tolerance = 0.1;       ///< [rad]

  /// Where odom lies in the simulator's world (planar), set by the bridge so
  /// the planner can work in world coordinates.
  double odom_x = 0.0, odom_y = 0.0, odom_yaw = 0.0;

  bool operator==(const NavigationRequest& o) const {
    return kind == o.kind && sequence == o.sequence && avoid == o.avoid && padding == o.padding &&
           max_vx == o.max_vx && max_vy == o.max_vy && max_wz == o.max_wz && goal_x == o.goal_x &&
           goal_y == o.goal_y && goal_yaw == o.goal_yaw && position_tolerance == o.position_tolerance &&
           yaw_tolerance == o.yaw_tolerance && odom_x == o.odom_x && odom_y == o.odom_y && odom_yaw == o.odom_yaw;
  }
  bool operator!=(const NavigationRequest& o) const { return !(*this == o); }
};

/// What the planner reports back.
struct NavigationStatus {
  enum class State : uint8_t {
    kIdle,      ///< no request, or a velocity without avoidance
    kMoving,    ///< following the velocity / on the way to the goal
    kNearGoal,  ///< close to the goal, settling
    kAtGoal,    ///< within the tolerances, standing still
    kBlocked,   ///< no way to the goal right now; keeps trying
  };
  State state = State::kIdle;
  /// NavigationRequest::sequence this status is for.
  uint32_t sequence = 0;

  bool operator==(const NavigationStatus& o) const { return state == o.state && sequence == o.sequence; }
  bool operator!=(const NavigationStatus& o) const { return !(*this == o); }
};

}  // namespace spot_sim
