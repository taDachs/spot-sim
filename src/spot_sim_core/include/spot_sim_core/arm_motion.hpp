#pragma once

#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include <bosdyn/api/arm_command.pb.h>
#include <bosdyn/api/robot_command.pb.h>

#include "spot_sim_core/arm.hpp"
#include "spot_sim_core/geometry.hpp"
#include "spot_sim_core/time.hpp"

namespace spot_sim {

/// Robot frames arm commands may be expressed in. "vision" and "gpe" are
/// the same as "odom" in the simulator (no drift, flat ground).
enum class RobotFrame { kBody, kFlatBody, kOdom };
std::optional<RobotFrame> ParseRobotFrame(const std::string& name);

/// What an arm motion needs to know about the robot each step.
struct ArmContext {
  ArmJoints joints;  ///< measured
  Pose odom_T_body;
};

/// body_T_frame for the robot frames above.
Pose BodyTFrame(RobotFrame frame, const Pose& odom_T_body);

/// One arm command being executed: turns an ArmCommand request into joint
/// setpoints over time, and reports its progress as ArmCommand feedback.
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
  enum class Kind { kNamed, kStop, kJointMove, kCartesian, kVelocity, kGaze };
  enum class Progress { kInProgress, kComplete, kStalled, kTimedOut };

  struct Rejection {
    bosdyn::api::RobotCommandResponse::Status status;
    std::string message;
  };

  /// Default speeds when a request gives none.
  static constexpr double kJointSpeed = 1.0;     ///< [rad/s]
  static constexpr double kLinearSpeed = 0.5;    ///< [m/s]
  static constexpr double kAngularSpeed = 1.0;   ///< [rad/s]
  /// Setpoint tracking speed of the arm plugin for time-parametrized motions.
  static constexpr double kTrackingSpeed = 10.0;  ///< [rad/s]

  /// Checks `request` and plans the motion from the robot's current state.
  static std::variant<ArmMotion, Rejection> Create(const bosdyn::api::ArmCommand::Request& request,
                                                   const ArmContext& context, RobotTime now, bool clock_synced);

  /// Move to fixed joints (named positions, internal stow) / stay put.
  static ArmMotion ToJoints(const ArmJoints& target,
                            bosdyn::api::NamedArmPositionsCommand::Positions position =
                                bosdyn::api::NamedArmPositionsCommand::POSITIONS_UNKNOWN);
  static ArmMotion Hold(const ArmJoints& joints);

  /// Advances to `now`: new setpoint, progress.
  void Step(RobotTime now, const ArmContext& context);

  Kind kind() const { return kind_; }
  Progress progress() const { return progress_; }
  const ArmJoints& setpoint() const { return setpoint_; }
  /// Speed limit for the arm plugin's tracking of the setpoint.
  double max_joint_speed() const { return max_joint_speed_; }
  std::optional<bosdyn::api::NamedArmPositionsCommand::Positions> named_position() const { return named_; }

  /// The kind-specific part of ArmCommand.Feedback (not its status).
  void FillFeedback(bosdyn::api::ArmCommand::Feedback* feedback) const;

 private:
  explicit ArmMotion(Kind kind) : kind_(kind) {}

  struct JointWaypoint {
    double t;  ///< robot time [s]
    ArmJoints joints;
  };
  struct PoseWaypoint {
    double t;
    Pose pose;  ///< task_T_tool
  };

  static std::variant<ArmMotion, Rejection> JointMove(const bosdyn::api::ArmJointMoveCommand::Request& request,
                                                      const ArmContext& context, RobotTime now);
  static std::variant<ArmMotion, Rejection> Cartesian(const bosdyn::api::ArmCartesianCommand::Request& request,
                                                      const ArmContext& context, RobotTime now);
  static std::variant<ArmMotion, Rejection> Velocity(const bosdyn::api::ArmVelocityCommand::Request& request,
                                                     const ArmContext& context, RobotTime now, bool clock_synced);
  static std::variant<ArmMotion, Rejection> Gaze(const bosdyn::api::GazeCommand::Request& request,
                                                 const ArmContext& context, RobotTime now);

  /// IK for a tool pose in the body frame; keeps the setpoint (and flags it)
  /// when out of reach.
  bool SolveTool(const Pose& body_T_tool, const ArmJoints& near);
  /// Done once the trajectory has ended and the arm got to the setpoint.
  void FinishWhenReached(double now_s, const ArmContext& context);

  Kind kind_;
  Progress progress_ = Progress::kInProgress;
  ArmJoints setpoint_;
  double max_joint_speed_ = kJointSpeed;
  std::optional<bosdyn::api::NamedArmPositionsCommand::Positions> named_;
  bool modified_ = false;   ///< the planner had to stretch the timing or clamp joints
  bool reachable_ = true;   ///< the latest target had an IK solution
  double end_s_ = 0.0;      ///< when the trajectory (or velocity command) ends [s robot time]
  double now_s_ = 0.0;      ///< latest step [s robot time]

  std::vector<JointWaypoint> joint_waypoints_;
  std::vector<PoseWaypoint> pose_waypoints_;
  RobotFrame root_ = RobotFrame::kBody;
  Pose root_T_task_;
  Pose wrist_T_tool_;

  // Velocity.
  bool cylindrical_ = true;
  Vec3 linear_;             ///< cylindrical (r, theta, z rates) or Cartesian in frame_
  std::optional<RobotFrame> velocity_frame_;  ///< nullopt: the hand frame
  Vec3 angular_in_hand_;
  double max_linear_speed_ = kLinearSpeed;
  Pose body_T_hand_target_;
  double last_s_ = 0.0;

  // Gaze.
  RobotFrame target_frame_ = RobotFrame::kBody;
  Vec3 target_;                    ///< in target_frame_
  std::optional<RobotFrame> tool_frame_;
  Pose tool_pose_;                 ///< in tool_frame_, or body_T_tool when held
  double distance_to_goal_ = 0.0;  ///< Cartesian / gaze feedback [m]
  double angle_to_goal_ = 0.0;     ///< [rad]
};

}  // namespace spot_sim
