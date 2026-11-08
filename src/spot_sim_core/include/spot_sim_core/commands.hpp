#pragma once

#include <chrono>
#include <cstdint>
#include <map>
#include <optional>

#include <bosdyn/api/estop.pb.h>
#include <bosdyn/api/lease.pb.h>
#include <bosdyn/api/robot_command.pb.h>

#include "spot_sim_core/locomotion.hpp"
#include "spot_sim_core/power.hpp"
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
/// LocomotionCommand for the base, and derives command feedback from the
/// base's LocomotionStatus.
///
/// Supported so far: mobility stand / sit / stop / se2_velocity, full-body
/// stop and safe power off. Also implements the robot-side behaviors that sequence
/// locomotion and power: safe power off (sit, then cut) and the
/// SETTLE_THEN_CUT E-Stop (sit, then cut, or cut after a timeout).
///
/// Not thread-safe; Robot serializes access.
class CommandSystem {
 public:
  explicit CommandSystem(std::chrono::nanoseconds settle_timeout = std::chrono::seconds(5));

  /// Handles RobotCommand after the lease was checked (`lease` is its result).
  /// `clock_synced`: the request's clock_identifier has an established time
  /// sync. `odom_yaw`: current body heading in odom/vision, to convert
  /// velocities given in those frames.
  bosdyn::api::RobotCommandResponse Command(const bosdyn::api::RobotCommandRequest& request,
                                            const bosdyn::api::LeaseUseResult& lease, bool motors_on,
                                            bool clock_synced, RobotTime now, double odom_yaw = 0.0);

  bosdyn::api::RobotCommandFeedbackResponse Feedback(const bosdyn::api::RobotCommandFeedbackRequest& request,
                                                     const LocomotionStatus& status) const;

  /// Advances command-driven sequences. Call every simulation step after the
  /// power system stepped.
  void Step(RobotTime now, bosdyn::api::EstopStopLevel estop_level, PowerSystem& power,
            const LocomotionStatus& status);

  /// Current target for the locomotion base.
  LocomotionCommand Locomotion(bool motors_on) const;

 private:
  enum class Kind { kStand, kSit, kMobilityStop, kVelocity, kFullBodyStop, kSafePowerOff };

  struct Record {
    Kind kind;
    uint32_t sequence = 0;
    bool overridden = false;
    bool done = false;       ///< safe power off completed
    bool timed_out = false;  ///< velocity command reached its end time
    RobotTime end_time{0};   ///< velocity commands
  };

  bosdyn::api::RobotCommandResponse Velocity(const bosdyn::api::MobilityCommand::Request& mobility,
                                             bool clock_synced, RobotTime now, double odom_yaw,
                                             bosdyn::api::RobotCommandResponse response);

  uint32_t Issue(Kind kind, LocomotionMode mode);
  void OverrideActive();
  static bool Reached(const Record& record, const LocomotionStatus& status, LocomotionState state);

  std::chrono::nanoseconds settle_timeout_;
  std::map<uint32_t, Record> records_;
  uint32_t next_id_ = 1;
  LocomotionCommand locomotion_;
  std::optional<uint32_t> active_;  ///< command currently driving locomotion
  std::optional<RobotTime> settling_since_;
  uint32_t settle_sequence_ = 0;
};

}  // namespace spot_sim
