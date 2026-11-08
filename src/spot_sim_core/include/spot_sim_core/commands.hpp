#pragma once

#include <chrono>
#include <cstdint>
#include <map>
#include <optional>
#include <string>

#include <bosdyn/api/arm_command.pb.h>
#include <bosdyn/api/estop.pb.h>
#include <bosdyn/api/gripper_command.pb.h>
#include <bosdyn/api/lease.pb.h>
#include <bosdyn/api/robot_command.pb.h>
#include <bosdyn/api/spot/robot_command.pb.h>

#include "spot_sim_core/arm.hpp"
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

/// RobotCommandService semantics: validates commands, turns them into a
/// LocomotionCommand for the base (and an ArmCommand for the arm), and
/// derives command feedback from the base's LocomotionStatus and the arm's
/// measured joints.
///
/// Supported so far: mobility stand / sit / stop / se2_velocity (with
/// MobilityParams body_control posture and velocity limits), full-body stop,
/// safe power off and self-right; with an arm, named arm positions (stow,
/// ready, carry), arm stop and the claw gripper.
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

  /// `odom_yaw`: current body heading in odom, to express se2 velocities
  /// given in odom/vision in the body frame.
  bosdyn::api::RobotCommandResponse Command(const bosdyn::api::RobotCommandRequest& request,
                                            const bosdyn::api::LeaseUseResult& lease, bool motors_on,
                                            bool clock_synced, RobotTime now, double odom_yaw = 0.0);

  bosdyn::api::RobotCommandFeedbackResponse Feedback(const bosdyn::api::RobotCommandFeedbackRequest& request,
                                                     const LocomotionStatus& status) const;

  /// Advances command execution: expires velocity commands, sequences safe
  /// power off and SETTLE_THEN_CUT, tracks arm progress (`arm`: the arm as
  /// measured, robots with an arm only).
  void Step(RobotTime now, bosdyn::api::EstopStopLevel estop_level, PowerSystem& power,
            const LocomotionStatus& status, const std::optional<ArmSample>& arm = std::nullopt);

  /// Current target for the locomotion base.
  LocomotionCommand Locomotion(bool motors_on) const;

  /// Current target for the arm.
  ArmCommand Arm(bool motors_on) const;

 private:
  enum class Kind { kStand, kSit, kMobilityStop, kVelocity, kFullBodyStop, kSafePowerOff, kSelfRight };
  enum class ArmKind { kNamed, kStop };

  struct ArmPart {
    ArmKind kind = ArmKind::kStop;
    bool overridden = false;
    bool reached = false;  ///< the arm got to the target while this command was active
  };
  struct GripperPart {
    bool overridden = false;
    bool reached = false;
  };
  struct Record {
    bool has_body = false;   ///< a mobility or full-body command
    Kind kind = Kind::kStand;
    uint32_t sequence = 0;
    bool overridden = false;
    bool done = false;       ///< safe power off completed
    bool timed_out = false;  ///< velocity command reached its end time
    RobotTime end_time{0};   ///< velocity commands
    std::optional<ArmPart> arm;
    std::optional<GripperPart> gripper;
  };

  bosdyn::api::RobotCommandResponse Mobility(const bosdyn::api::MobilityCommand::Request& mobility, bool motors_on,
                                             bool clock_synced, RobotTime now, double odom_yaw,
                                             bosdyn::api::RobotCommandResponse response);
  bosdyn::api::RobotCommandResponse Velocity(const bosdyn::api::SE2VelocityCommand::Request& request,
                                             const bosdyn::api::spot::MobilityParams& params, bool clock_synced,
                                             RobotTime now, double odom_yaw,
                                             bosdyn::api::RobotCommandResponse response);
  /// Validates an arm or gripper request; an error message if it cannot be executed.
  static std::optional<std::string> CheckArm(const bosdyn::api::ArmCommand::Request& request);
  static std::optional<std::string> CheckGripper(const bosdyn::api::GripperCommand::Request& request);
  void StartArm(uint32_t id, const bosdyn::api::ArmCommand::Request& request);
  void StartGripper(uint32_t id, const bosdyn::api::GripperCommand::Request& request);

  void SetPosture(const BodyPosture& posture);

  uint32_t NewRecord();
  uint32_t Issue(Kind kind, LocomotionMode mode);
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
  static bool Reached(const Record& record, const LocomotionStatus& status, LocomotionState state);

  std::chrono::nanoseconds settle_timeout_;
  std::map<uint32_t, Record> records_;
  uint32_t next_id_ = 1;
  LocomotionCommand locomotion_;
  std::optional<uint32_t> active_;  ///< command currently driving locomotion
  std::optional<RobotTime> settling_since_;
  uint32_t settle_sequence_ = 0;

  bool has_arm_ = false;
  ArmCommand arm_;  ///< arm and gripper setpoints (motors_on filled in by Arm())
  std::optional<uint32_t> active_arm_;      ///< command currently driving the arm
  std::optional<bosdyn::api::NamedArmPositionsCommand::Positions> active_named_;
  std::optional<uint32_t> active_gripper_;  ///< command currently driving the gripper
  std::optional<ArmSample> measured_arm_;
};

}  // namespace spot_sim
