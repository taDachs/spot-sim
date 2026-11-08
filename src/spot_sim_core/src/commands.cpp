#include "spot_sim_core/commands.hpp"

namespace spot_sim {

namespace api = bosdyn::api;

namespace {
constexpr std::size_t kMaxRememberedCommands = 1024;
}

CommandSystem::CommandSystem(std::chrono::nanoseconds settle_timeout) : settle_timeout_(settle_timeout) {}

void CommandSystem::OverrideActive() {
  if (active_) {
    records_[*active_].overridden = true;
    active_.reset();
  }
}

uint32_t CommandSystem::Issue(Kind kind, LocomotionMode mode) {
  OverrideActive();
  locomotion_.mode = mode;
  ++locomotion_.sequence;
  const uint32_t id = next_id_++;
  records_[id] = Record{kind, locomotion_.sequence};
  while (records_.size() > kMaxRememberedCommands) {
    records_.erase(records_.begin());
  }
  active_ = id;
  return id;
}

api::RobotCommandResponse CommandSystem::Command(const api::RobotCommandRequest& request,
                                                 const api::LeaseUseResult& lease, bool motors_on,
                                                 bool /*clock_synced*/, RobotTime /*now*/) {
  api::RobotCommandResponse response;
  *response.mutable_lease_use_result() = lease;
  if (lease.status() != api::LeaseUseResult::STATUS_OK) {
    return response;
  }
  const auto reject = [&](api::RobotCommandResponse::Status status, const char* message) {
    response.set_status(status);
    response.set_message(message);
    return response;
  };
  const auto accept = [&](Kind kind, LocomotionMode mode) {
    response.set_robot_command_id(Issue(kind, mode));
    response.set_status(api::RobotCommandResponse::STATUS_OK);
    return response;
  };

  const auto& command = request.command();
  if (command.has_full_body_command()) {
    const auto& full_body = command.full_body_command();
    if (full_body.has_stop_request()) {
      return accept(Kind::kFullBodyStop, LocomotionMode::kStop);
    }
    if (full_body.has_safe_power_off_request()) {
      const uint32_t id = Issue(Kind::kSafePowerOff, LocomotionMode::kSit);
      if (!motors_on) {
        records_[id].done = true;
      }
      response.set_robot_command_id(id);
      response.set_status(api::RobotCommandResponse::STATUS_OK);
      return response;
    }
    return reject(api::RobotCommandResponse::STATUS_UNSUPPORTED, "full body command not simulated");
  }

  if (command.has_synchronized_command()) {
    const auto& sync = command.synchronized_command();
    if (sync.has_arm_command() || sync.has_gripper_command()) {
      return reject(api::RobotCommandResponse::STATUS_UNSUPPORTED, "this robot has no arm");
    }
    if (!sync.has_mobility_command()) {
      return reject(api::RobotCommandResponse::STATUS_INVALID_REQUEST, "empty synchronized command");
    }
    const auto& mobility = sync.mobility_command();
    if (mobility.has_stop_request()) {
      return accept(Kind::kMobilityStop, LocomotionMode::kStop);
    }
    if (!motors_on) {
      return reject(api::RobotCommandResponse::STATUS_NOT_POWERED_ON, "motors are off");
    }
    if (mobility.has_stand_request()) {
      return accept(Kind::kStand, LocomotionMode::kStand);
    }
    if (mobility.has_sit_request()) {
      return accept(Kind::kSit, LocomotionMode::kSit);
    }
    return reject(api::RobotCommandResponse::STATUS_UNSUPPORTED, "mobility command not simulated yet");
  }

  return reject(api::RobotCommandResponse::STATUS_INVALID_REQUEST, "empty command");
}

bool CommandSystem::Reached(const Record& record, const LocomotionStatus& status, LocomotionState state) {
  return status.state == state && status.reached_sequence >= record.sequence;
}

api::RobotCommandFeedbackResponse CommandSystem::Feedback(const api::RobotCommandFeedbackRequest& request,
                                                          const LocomotionStatus& status) const {
  api::RobotCommandFeedbackResponse response;
  const auto it = records_.find(request.robot_command_id());
  if (it == records_.end()) {
    return response;
  }
  const Record& record = it->second;
  const auto processing =
      record.overridden ? api::RobotCommandFeedbackStatus::STATUS_COMMAND_OVERRIDDEN
                        : api::RobotCommandFeedbackStatus::STATUS_PROCESSING;

  auto* feedback = response.mutable_feedback();
  switch (record.kind) {
    case Kind::kStand: {
      auto* mobility = feedback->mutable_synchronized_feedback()->mutable_mobility_command_feedback();
      mobility->set_status(processing);
      auto* stand = mobility->mutable_stand_feedback();
      stand->set_status(Reached(record, status, LocomotionState::kStanding)
                            ? api::StandCommand::Feedback::STATUS_IS_STANDING
                            : api::StandCommand::Feedback::STATUS_IN_PROGRESS);
      stand->set_standing_state(api::StandCommand::Feedback::STANDING_CONTROLLED);
      break;
    }
    case Kind::kSit: {
      auto* mobility = feedback->mutable_synchronized_feedback()->mutable_mobility_command_feedback();
      mobility->set_status(processing);
      mobility->mutable_sit_feedback()->set_status(Reached(record, status, LocomotionState::kSitting)
                                                       ? api::SitCommand::Feedback::STATUS_IS_SITTING
                                                       : api::SitCommand::Feedback::STATUS_IN_PROGRESS);
      break;
    }
    case Kind::kMobilityStop: {
      auto* mobility = feedback->mutable_synchronized_feedback()->mutable_mobility_command_feedback();
      mobility->set_status(processing);
      mobility->mutable_stop_feedback();
      break;
    }
    case Kind::kFullBodyStop: {
      auto* full_body = feedback->mutable_full_body_feedback();
      full_body->set_status(processing);
      full_body->mutable_stop_feedback();
      break;
    }
    case Kind::kSafePowerOff: {
      auto* full_body = feedback->mutable_full_body_feedback();
      full_body->set_status(record.done ? api::RobotCommandFeedbackStatus::STATUS_PROCESSING : processing);
      full_body->mutable_safe_power_off_feedback()->set_status(
          record.done ? api::SafePowerOffCommand::Feedback::STATUS_POWERED_OFF
                      : api::SafePowerOffCommand::Feedback::STATUS_IN_PROGRESS);
      break;
    }
  }
  return response;
}

void CommandSystem::Step(RobotTime now, api::EstopStopLevel estop_level, PowerSystem& power,
                         const LocomotionStatus& status) {
  // Safe power off: once sitting, cut motor power.
  if (active_) {
    Record& record = records_[*active_];
    if (record.kind == Kind::kSafePowerOff && !record.done && power.MotorsOn() &&
        Reached(record, status, LocomotionState::kSitting)) {
      power.Cut();
      record.done = true;
    }
  }

  // SETTLE_THEN_CUT E-Stop: sit down, then cut (or cut after a timeout).
  if (estop_level == api::ESTOP_LEVEL_SETTLE_THEN_CUT && power.MotorsOn()) {
    if (!settling_since_) {
      settling_since_ = now;
      Issue(Kind::kSit, LocomotionMode::kSit);
      settle_sequence_ = locomotion_.sequence;
    }
    const bool seated = status.state == LocomotionState::kSitting && status.reached_sequence >= settle_sequence_;
    if (seated || now - *settling_since_ >= settle_timeout_) {
      power.Cut(bosdyn::api::STATUS_ESTOPPED);
    }
  }
  if (estop_level == api::ESTOP_LEVEL_NONE || !power.MotorsOn()) {
    settling_since_.reset();
  }

  // Without motor power nothing is being executed any more.
  if (!power.MotorsOn() && active_) {
    Record& record = records_[*active_];
    if (record.kind != Kind::kSafePowerOff) {
      OverrideActive();
    } else if (!record.done) {
      // Power went away by other means (E-Stop); the robot is off anyway.
      record.done = true;
    }
  }
  if (!power.MotorsOn()) {
    locomotion_.mode = LocomotionMode::kSit;
  }
}

LocomotionCommand CommandSystem::Locomotion(bool motors_on) const {
  LocomotionCommand command = locomotion_;
  command.motors_on = motors_on;
  return command;
}

}  // namespace spot_sim
