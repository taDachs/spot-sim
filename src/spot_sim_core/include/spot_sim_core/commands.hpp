#pragma once

#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <variant>

#include <bosdyn/api/arm_command.pb.h>
#include <bosdyn/api/estop.pb.h>
#include <bosdyn/api/gripper_command.pb.h>
#include <bosdyn/api/lease.pb.h>
#include <bosdyn/api/robot_command.pb.h>
#include <bosdyn/api/spot/robot_command.pb.h>

#include "spot_sim_core/arm.hpp"
#include "spot_sim_core/arm_motion.hpp"
#include "spot_sim_core/command_request.hpp"
#include "spot_sim_core/locomotion.hpp"
#include "spot_sim_core/power.hpp"
#include "spot_sim_core/robot_state.hpp"
#include "spot_sim_core/stance.hpp"
#include "spot_sim_core/time.hpp"

namespace spot_sim {

/// Planar velocity bounds in the gravity-aligned body frame [m/s, rad/s].
struct VelocityLimits {
  double max_x = 1.6, max_y = 0.6, max_rot = 1.2;
  double min_x = -1.0, min_y = -0.6, min_rot = -1.2;

  /// Approximation of the real robot's own limits; requests beyond them are
  /// clamped, as MobilityParams.vel_limit can only tighten them.
  static VelocityLimits Robot() { return {}; }
};

/// What the robot knows about itself when a command arrives.
struct CommandContext {
  bool motors_on = false;
  bool clock_synced = false;  ///< the request's clock_identifier has an established time sync
  RobotTime now{0};
  /// Current body heading in odom, to express se2 velocities given in
  /// odom/vision in the body frame.
  double odom_yaw = 0.0;
};

/// RobotCommandService semantics: validates commands, turns them into a
/// LocomotionCommand for the base (and an ArmCommand for the arm), and
/// derives command feedback from the base's LocomotionStatus and the arm's
/// measured joints.
///
/// Supported so far: mobility stand / sit / stop / se2_velocity (with
/// MobilityParams body_control posture and velocity limits), full-body stop,
/// safe power off and self-right; with an arm, named arm positions (stow,
/// ready, carry), arm stop, joint moves, Cartesian moves, arm velocity and
/// gaze (see ArmMotion), and the claw gripper.
///
/// Like on the robot, a synchronized command has independent mobility, arm
/// and gripper parts: a new command overrides only the parts it contains
/// (an arm-only command leaves walking alone); full-body commands override
/// all of them.
///
/// Also implements the robot-side behaviors that sequence locomotion, arm and
/// power: sitting down (and safe power off) stows the arm first, safe power
/// off sits, then cuts, and the SETTLE_THEN_CUT E-Stop sits, then cuts (or
/// cuts after a timeout).
///
/// Not thread-safe; Robot serializes access.
class CommandSystem {
 public:
  explicit CommandSystem(std::chrono::nanoseconds settle_timeout = std::chrono::seconds(5));

  /// The robot has an arm: arm and gripper commands are accepted.
  void EnableArm() { has_arm_ = true; }

  bosdyn::api::RobotCommandResponse Command(const bosdyn::api::RobotCommandRequest& request,
                                            const bosdyn::api::LeaseUseResult& lease, const CommandContext& context);

  bosdyn::api::RobotCommandFeedbackResponse Feedback(const bosdyn::api::RobotCommandFeedbackRequest& request,
                                                     const LocomotionStatus& status) const;

  /// Advances command execution: expires velocity commands, sequences safe
  /// power off and SETTLE_THEN_CUT, moves the arm along (`arm`: the arm as
  /// measured, robots with an arm only; `odom_T_body`: the body pose).
  void Step(RobotTime now, bosdyn::api::EstopStopLevel estop_level, PowerSystem& power,
            const LocomotionStatus& status, const std::optional<ArmSample>& arm = std::nullopt,
            const Pose& odom_T_body = Pose{});

  /// Current target for the locomotion base.
  LocomotionCommand Locomotion(bool motors_on) const;

  /// Current target for the arm.
  ArmCommand Arm(bool motors_on) const;

 private:
  /// The mobility or full-body part of a command, with the state that only
  /// some kinds of command have.
  struct BodyPart {
    struct Stand {};
    struct Sit {};
    struct MobilityStop {};
    struct Velocity {
      RobotTime end_time{0};
      bool timed_out = false;  ///< reached its end time
    };
    struct FullBodyStop {};
    struct SafePowerOff {
      bool done = false;  ///< motor power is off
    };
    struct SelfRight {};
    using Kind = std::variant<Stand, Sit, MobilityStop, Velocity, FullBodyStop, SafePowerOff, SelfRight>;

    Kind kind;
    uint32_t sequence = 0;  ///< LocomotionCommand sequence it issued
    bool overridden = false;
  };
  struct ArmPart {
    std::shared_ptr<const ArmMotion> motion;
    bool overridden = false;
  };
  struct GripperPart {
    bool overridden = false;
    bool reached = false;
  };
  struct Record {
    std::optional<BodyPart> body;
    std::optional<ArmPart> arm;
    std::optional<GripperPart> gripper;
  };

  /// The id of the command a request started, or why it was refused.
  using Issued = std::variant<uint32_t, Rejection>;

  Issued FullBody(const bosdyn::api::FullBodyCommand::Request& request, const CommandContext& context);
  Issued Synchronized(const bosdyn::api::SynchronizedCommand::Request& request, const CommandContext& context);
  Issued Mobility(const bosdyn::api::MobilityCommand::Request& request, const CommandContext& context);
  Issued Velocity(const bosdyn::api::SE2VelocityCommand::Request& request,
                  const bosdyn::api::spot::MobilityParams& params, const CommandContext& context);
  /// Validates a gripper request: why it cannot be executed, if it cannot.
  static std::optional<Rejection> CheckGripper(const bosdyn::api::GripperCommand::Request& request);
  ArmContext CurrentArmContext() const;
  /// Makes `motion` drive the arm (recorded under command `id` unless 0).
  void StartArm(uint32_t id, std::unique_ptr<ArmMotion> motion);
  void ApplyArm();
  void StartGripper(uint32_t id, const bosdyn::api::GripperCommand::Request& request);

  void SetPosture(const BodyPosture& posture);

  uint32_t NewRecord();
  /// Starts a mobility or full-body command: a new LocomotionCommand.
  uint32_t Issue(BodyPart::Kind kind, LocomotionMode mode);
  /// The body part of the command driving locomotion, if any.
  BodyPart* ActiveBody();
  void OverrideActive();
  void OverrideArm();
  void OverrideGripper();
  /// Keep the arm (gripper) where it is.
  void HoldArm();
  void HoldGripper();
  /// Stow the arm (before sitting down), unless a stow is already under way.
  void StowArm();
  bool ArmStowed() const;
  Record* Find(std::optional<uint32_t> id);
  /// The base reached `state` for LocomotionCommand `sequence` (or a later one).
  static bool Reached(uint32_t sequence, const LocomotionStatus& status, LocomotionState state);

  std::chrono::nanoseconds settle_timeout_;
  std::map<uint32_t, Record> records_;
  uint32_t next_id_ = 1;
  LocomotionCommand locomotion_;
  std::optional<uint32_t> active_;  ///< command currently driving locomotion
  std::optional<RobotTime> settling_since_;
  uint32_t settle_sequence_ = 0;

  bool has_arm_ = false;
  ArmCommand arm_;  ///< arm and gripper setpoints (motors_on filled in by Arm())
  std::shared_ptr<ArmMotion> arm_motion_;  ///< what the arm is doing (also internal stow / hold)
  std::optional<uint32_t> active_arm_;      ///< command currently driving the arm
  std::optional<uint32_t> active_gripper_;  ///< command currently driving the gripper
  std::optional<ArmSample> measured_arm_;
  Pose odom_T_body_;
};

}  // namespace spot_sim
