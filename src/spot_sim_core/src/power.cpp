#include "spot_sim_core/power.hpp"

namespace spot_sim {

namespace api = bosdyn::api;

namespace {
constexpr std::size_t kMaxRememberedCommands = 256;
}

PowerSystem::PowerSystem(std::chrono::nanoseconds power_on_duration, std::chrono::nanoseconds power_off_duration)
    : on_duration_(power_on_duration), off_duration_(power_off_duration) {}

uint32_t PowerSystem::StartCommand(api::PowerCommandStatus status) {
  const uint32_t id = next_id_++;
  results_[id] = status;
  while (results_.size() > kMaxRememberedCommands) {
    results_.erase(results_.begin());
  }
  return id;
}

void PowerSystem::FinishPending(api::PowerCommandStatus status) {
  if (pending_id_ != 0) {
    results_[pending_id_] = status;
    pending_id_ = 0;
  }
}

api::PowerCommandResponse PowerSystem::Command(const api::PowerCommandRequest& request,
                                               api::EstopStopLevel estop_level, RobotTime now) {
  api::PowerCommandResponse response;
  const auto respond = [&](api::PowerCommandStatus status) {
    response.set_status(status);
    response.set_power_command_id(StartCommand(status));
    return response;
  };

  switch (request.request()) {
    case api::PowerCommandRequest::REQUEST_ON_MOTORS:
      if (estop_level != api::ESTOP_LEVEL_NONE) {
        return respond(api::STATUS_ESTOPPED);
      }
      if (phase_ == Phase::kOn) {
        return respond(api::STATUS_SUCCESS);
      }
      FinishPending(api::STATUS_OVERRIDDEN);
      if (phase_ != Phase::kPoweringOn) {
        phase_ = Phase::kPoweringOn;
        transition_end_ = now + on_duration_;
      }
      respond(api::STATUS_IN_PROGRESS);
      pending_id_ = response.power_command_id();
      return response;

    case api::PowerCommandRequest::REQUEST_OFF_MOTORS:
      if (phase_ == Phase::kOff) {
        return respond(api::STATUS_SUCCESS);
      }
      FinishPending(api::STATUS_OVERRIDDEN);
      if (phase_ != Phase::kPoweringOff) {
        phase_ = Phase::kPoweringOff;
        transition_end_ = now + off_duration_;
      }
      respond(api::STATUS_IN_PROGRESS);
      pending_id_ = response.power_command_id();
      return response;

    case api::PowerCommandRequest::REQUEST_ON_PAYLOAD_PORTS:
    case api::PowerCommandRequest::REQUEST_OFF_PAYLOAD_PORTS:
    case api::PowerCommandRequest::REQUEST_ON_WIFI_RADIO:
    case api::PowerCommandRequest::REQUEST_OFF_WIFI_RADIO:
      // Accepted, no simulated effect.
      return respond(api::STATUS_SUCCESS);

    default:
      // Turning the whole robot off or cycling it would end the simulation.
      return respond(api::STATUS_INTERNAL_ERROR);
  }
}

api::PowerCommandFeedbackResponse PowerSystem::Feedback(const api::PowerCommandFeedbackRequest& request) const {
  api::PowerCommandFeedbackResponse response;
  const auto it = results_.find(request.power_command_id());
  response.set_status(it == results_.end() ? api::STATUS_UNKNOWN : it->second);
  return response;
}

void PowerSystem::Cut(api::PowerCommandStatus reason_for_pending) {
  if (phase_ == Phase::kPoweringOn) {
    FinishPending(reason_for_pending);
  } else {
    // A pending power-off is fulfilled by cutting power.
    FinishPending(api::STATUS_SUCCESS);
  }
  phase_ = Phase::kOff;
}

void PowerSystem::Step(api::EstopStopLevel estop_level, RobotTime now) {
  // CUT stops immediately. SETTLE_THEN_CUT lets a powered robot sit down
  // first (CommandSystem cuts afterwards) but aborts a power-on in progress.
  const bool cut = estop_level != api::ESTOP_LEVEL_NONE && estop_level != api::ESTOP_LEVEL_SETTLE_THEN_CUT;
  const bool settle = estop_level == api::ESTOP_LEVEL_SETTLE_THEN_CUT;
  if ((cut && phase_ != Phase::kOff) || (settle && phase_ == Phase::kPoweringOn)) {
    Cut(api::STATUS_ESTOPPED);
    return;
  }
  if (phase_ == Phase::kPoweringOn && now >= transition_end_) {
    phase_ = Phase::kOn;
    FinishPending(api::STATUS_SUCCESS);
  } else if (phase_ == Phase::kPoweringOff && now >= transition_end_) {
    phase_ = Phase::kOff;
    FinishPending(api::STATUS_SUCCESS);
  }
}

api::PowerState::MotorPowerState PowerSystem::MotorState() const {
  switch (phase_) {
    case Phase::kOn:
      return api::PowerState::MOTOR_POWER_STATE_ON;
    case Phase::kPoweringOn:
      return api::PowerState::MOTOR_POWER_STATE_POWERING_ON;
    case Phase::kPoweringOff:
      return api::PowerState::MOTOR_POWER_STATE_POWERING_OFF;
    case Phase::kOff:
    default:
      return api::PowerState::MOTOR_POWER_STATE_OFF;
  }
}

}  // namespace spot_sim
