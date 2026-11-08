#pragma once

#include <chrono>
#include <cstdint>
#include <map>

#include <bosdyn/api/estop.pb.h>
#include <bosdyn/api/power.pb.h>
#include <bosdyn/api/robot_state.pb.h>

#include "spot_sim_core/time.hpp"

namespace spot_sim {

/// Motor power state machine. Powering on/off takes a little robot (sim)
/// time, during which feedback reports STATUS_IN_PROGRESS. Any E-Stop level
/// other than NONE cuts motor power.
///
/// Lease checks happen before Command() is called. Not thread-safe; Robot
/// serializes access.
class PowerSystem {
 public:
  explicit PowerSystem(std::chrono::nanoseconds power_on_duration = std::chrono::seconds(1),
                       std::chrono::nanoseconds power_off_duration = std::chrono::milliseconds(500));

  /// Handles PowerCommand. Fills status and power_command_id.
  bosdyn::api::PowerCommandResponse Command(const bosdyn::api::PowerCommandRequest& request,
                                            bosdyn::api::EstopStopLevel estop_level, RobotTime now);

  bosdyn::api::PowerCommandFeedbackResponse Feedback(const bosdyn::api::PowerCommandFeedbackRequest& request) const;

  /// Advances transitions and applies the E-Stop. Call every simulation step.
  void Step(bosdyn::api::EstopStopLevel estop_level, RobotTime now);

  /// Cut motor power immediately (E-Stop, safe power off after sitting).
  void Cut(bosdyn::api::PowerCommandStatus reason_for_pending = bosdyn::api::STATUS_OVERRIDDEN);

  bosdyn::api::PowerState::MotorPowerState MotorState() const;
  bool MotorsOn() const { return phase_ == Phase::kOn; }

 private:
  enum class Phase { kOff, kPoweringOn, kOn, kPoweringOff };

  uint32_t StartCommand(bosdyn::api::PowerCommandStatus status);
  void FinishPending(bosdyn::api::PowerCommandStatus status);

  std::chrono::nanoseconds on_duration_;
  std::chrono::nanoseconds off_duration_;
  Phase phase_ = Phase::kOff;
  RobotTime transition_end_{0};
  uint32_t next_id_ = 1;
  uint32_t pending_id_ = 0;  ///< command waiting for the current transition
  std::map<uint32_t, bosdyn::api::PowerCommandStatus> results_;
};

}  // namespace spot_sim
