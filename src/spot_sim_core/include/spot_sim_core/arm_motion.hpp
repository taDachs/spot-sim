#pragma once

#include <memory>
#include <optional>
#include <variant>

#include <bosdyn/api/arm_command.pb.h>
#include <bosdyn/api/robot_command.pb.h>

#include "spot_sim_core/arm.hpp"
#include "spot_sim_core/command_request.hpp"
#include "spot_sim_core/geometry.hpp"
#include "spot_sim_core/robot_frames.hpp"
#include "spot_sim_core/time.hpp"

namespace spot_sim {

/// What an arm motion needs to know about the robot each step.
struct ArmContext {
  ArmJoints joints;  ///< measured
  Pose odom_T_body;
};

/// One arm command being executed: turns an ArmCommand request into joint
/// setpoints over time, and reports its progress as ArmCommand feedback.
/// Each kind of command is its own subclass (arm_motion.cpp):
///
/// - Named positions and arm stop: a fixed joint target.
/// - Joint moves: the joint trajectory, linearly interpolated in time.
/// - Cartesian moves: the tool pose trajectory (linear in position, slerp in
///   orientation) in its root frame, solved with IK every step, so a pose in
///   odom stays put in the world while the body moves.
/// - Velocity: integrates the hand velocity (cylindrical about the shoulder or
///   Cartesian in a frame) until end_time; stops at the edge of the workspace.
/// - Gaze: points the tool's x axis at a target point, the hand at a fixed
///   place relative to the body (or a given tool pose), level (no roll).
///
/// Trajectory points come no faster than the speed limits allow (the
/// request's, or defaults): times that are too short are stretched (planner
/// status MODIFIED). Targets the arm cannot reach make the motion STALLED.
class ArmMotion {
 public:
  enum class Progress { kInProgress, kComplete, kStalled, kTimedOut };

  /// Default speeds when a request gives none.
  static constexpr double kJointSpeed = 1.0;     ///< [rad/s]
  static constexpr double kLinearSpeed = 0.5;    ///< [m/s]
  static constexpr double kAngularSpeed = 1.0;   ///< [rad/s]
  /// Setpoint tracking speed of the arm plugin for time-parametrized motions.
  static constexpr double kTrackingSpeed = 10.0;  ///< [rad/s]

  virtual ~ArmMotion() = default;

  /// Checks `request` and plans the motion from the robot's current state.
  static std::variant<std::unique_ptr<ArmMotion>, Rejection> Create(const bosdyn::api::ArmCommand::Request& request,
                                                                    const ArmContext& context, RobotTime now,
                                                                    bool clock_synced);

  /// Move to fixed joints (named positions, internal stow) / stay put.
  static std::unique_ptr<ArmMotion> ToJoints(const ArmJoints& target,
                                             bosdyn::api::NamedArmPositionsCommand::Positions position =
                                                 bosdyn::api::NamedArmPositionsCommand::POSITIONS_UNKNOWN);
  static std::unique_ptr<ArmMotion> Hold(const ArmJoints& joints);

  /// Advances to `now`: new setpoint, progress.
  virtual void Step(RobotTime now, const ArmContext& context) = 0;

  /// The kind-specific part of ArmCommand.Feedback (not its status).
  virtual void FillFeedback(bosdyn::api::ArmCommand::Feedback* feedback) const = 0;

  /// The named position this motion goes to (named positions, internal stow).
  virtual std::optional<bosdyn::api::NamedArmPositionsCommand::Positions> named_position() const {
    return std::nullopt;
  }
  /// An arm stop: the arm stays where it was.
  virtual bool is_hold() const { return false; }

  Progress progress() const { return progress_; }
  const ArmJoints& setpoint() const { return setpoint_; }
  /// Speed limit for the arm plugin's tracking of the setpoint.
  double max_joint_speed() const { return max_joint_speed_; }

 protected:
  ArmMotion(const ArmJoints& setpoint, double max_joint_speed)
      : setpoint_(setpoint), max_joint_speed_(max_joint_speed) {}

  /// IK for a tool pose in the body frame; keeps the setpoint (and flags it
  /// unreachable) when out of reach.
  bool SolveTool(const Pose& body_T_tool, const Pose& wrist_T_tool);
  /// Done once `end_s` has passed and the arm got to the setpoint; stalled
  /// if the latest target was out of reach.
  void FinishWhenReached(double now_s, double end_s, const ArmContext& context);

  Progress progress_ = Progress::kInProgress;
  ArmJoints setpoint_;
  double max_joint_speed_;
  bool reachable_ = true;  ///< the latest target had an IK solution
};

}  // namespace spot_sim
