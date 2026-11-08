#include "spot_sim_core/commands.hpp"

#include <algorithm>
#include <cmath>

#include <bosdyn/api/spot/robot_command.pb.h>

#include "spot_sim_core/stance.hpp"

namespace spot_sim {

namespace api = bosdyn::api;

namespace {
constexpr std::size_t kMaxRememberedCommands = 1024;

/// Tightens the robot's limits with MobilityParams.vel_limit.
VelocityLimits LimitsFrom(const api::spot::MobilityParams& params) {
  VelocityLimits limits = VelocityLimits::Robot();
  if (params.has_vel_limit()) {
    const auto& vel_limit = params.vel_limit();
    if (vel_limit.has_max_vel()) {
      limits.max_x = std::min(limits.max_x, vel_limit.max_vel().linear().x());
      limits.max_y = std::min(limits.max_y, vel_limit.max_vel().linear().y());
      limits.max_rot = std::min(limits.max_rot, vel_limit.max_vel().angular());
    }
    if (vel_limit.has_min_vel()) {
      limits.min_x = std::max(limits.min_x, vel_limit.min_vel().linear().x());
      limits.min_y = std::max(limits.min_y, vel_limit.min_vel().linear().y());
      limits.min_rot = std::max(limits.min_rot, vel_limit.min_vel().angular());
    }
  }
  return limits;
}

/// Body posture requested by body_control (base_offset_rt_footprint; its
/// final point, held). Anything else means the default posture. Saturated.
BodyPosture PostureFrom(const api::spot::MobilityParams& params) {
  if (!params.has_body_control() || !params.body_control().has_base_offset_rt_footprint()) {
    return {};
  }
  const auto& points = params.body_control().base_offset_rt_footprint().points();
  if (points.empty()) {
    return {};
  }
  const auto& pose = points.rbegin()->pose();
  const auto& r = pose.rotation();
  // An unset rotation (all zero) means no rotation.
  const Quat rotation =
      (r.w() == 0.0 && r.x() == 0.0 && r.y() == 0.0 && r.z() == 0.0) ? Quat{} : Quat{r.w(), r.x(), r.y(), r.z()}.Normalized();
  return BodyPosture{pose.position().z(), rotation.Roll(), rotation.Pitch(), rotation.Yaw()}.Clamped();
}

double Clamp(double value, double low, double high) {
  // An inconsistent limit (min > max) stops that axis rather than throwing.
  return low > high ? 0.0 : std::clamp(value, low, high);
}
}  // namespace

CommandSystem::CommandSystem(std::chrono::nanoseconds settle_timeout) : settle_timeout_(settle_timeout) {}

void CommandSystem::OverrideActive() {
  if (active_) {
    records_[*active_].overridden = true;
    active_.reset();
  }
}

uint32_t CommandSystem::Issue(Kind kind, LocomotionMode mode) {
  OverrideActive();
  const LocomotionCommand defaults;
  locomotion_.mode = mode;
  locomotion_.v_x = locomotion_.v_y = locomotion_.v_rot = 0.0;
  locomotion_.max_accel_x = defaults.max_accel_x;
  locomotion_.max_accel_y = defaults.max_accel_y;
  locomotion_.max_accel_rot = defaults.max_accel_rot;
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
                                                 bool clock_synced, RobotTime now, double odom_yaw) {
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
    if (full_body.has_selfright_request()) {
      // The simulated robot never falls over; self-righting ends lying down,
      // i.e. sitting.
      if (!motors_on) {
        return reject(api::RobotCommandResponse::STATUS_NOT_POWERED_ON, "motors are off");
      }
      auto accepted = accept(Kind::kSelfRight, LocomotionMode::kSit);
      SetPosture({});
      return accepted;
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
    api::spot::MobilityParams params;
    if (mobility.has_params() && !mobility.params().UnpackTo(&params)) {
      return reject(api::RobotCommandResponse::STATUS_INVALID_REQUEST, "params is not a spot.MobilityParams");
    }
    // Locomotion hints and stairs mode are accepted; the kinematic base walks
    // the same way regardless.
    if (mobility.has_stand_request()) {
      auto accepted = accept(Kind::kStand, LocomotionMode::kStand);
      SetPosture(PostureFrom(params));
      return accepted;
    }
    if (mobility.has_sit_request()) {
      auto accepted = accept(Kind::kSit, LocomotionMode::kSit);
      SetPosture({});
      return accepted;
    }
    if (mobility.has_se2_velocity_request()) {
      return Velocity(mobility.se2_velocity_request(), params, clock_synced, now, odom_yaw, std::move(response));
    }
    return reject(api::RobotCommandResponse::STATUS_UNSUPPORTED, "mobility command not simulated yet");
  }

  return reject(api::RobotCommandResponse::STATUS_INVALID_REQUEST, "empty command");
}

void CommandSystem::SetPosture(const BodyPosture& posture) {
  locomotion_.body_height_offset = posture.height_offset;
  locomotion_.body_roll = posture.roll;
  locomotion_.body_pitch = posture.pitch;
  locomotion_.body_yaw = posture.yaw;
}

api::RobotCommandResponse CommandSystem::Velocity(const api::SE2VelocityCommand::Request& request,
                                                  const api::spot::MobilityParams& params, bool clock_synced,
                                                  RobotTime now, double odom_yaw, api::RobotCommandResponse response) {
  const auto reject = [&](api::RobotCommandResponse::Status status, const char* message) {
    response.set_status(status);
    response.set_message(message);
    return response;
  };
  if (!clock_synced) {
    return reject(api::RobotCommandResponse::STATUS_NO_TIMESYNC, "clock_identifier has no established time sync");
  }
  if (!request.has_end_time()) {
    return reject(api::RobotCommandResponse::STATUS_INVALID_REQUEST, "se2_velocity_request requires an end_time");
  }
  const RobotTime end_time = FromTimestamp(request.end_time());
  if (end_time <= now) {
    return reject(api::RobotCommandResponse::STATUS_EXPIRED, "end_time has already passed");
  }

  // Express the velocity in the gravity-aligned body frame.
  const auto& frame = request.se2_frame_name();
  double v_x = request.velocity().linear().x();
  double v_y = request.velocity().linear().y();
  if (frame == "odom" || frame == "vision") {
    const double c = std::cos(odom_yaw), s = std::sin(odom_yaw);
    const double x = c * v_x + s * v_y;
    v_y = -s * v_x + c * v_y;
    v_x = x;
  } else if (frame != "body" && frame != "flat_body") {
    return reject(api::RobotCommandResponse::STATUS_UNKNOWN_FRAME, "se2_frame_name must be body, flat_body, odom or vision");
  }

  const VelocityLimits limits = LimitsFrom(params);

  const uint32_t id = Issue(Kind::kVelocity, LocomotionMode::kVelocity);
  records_[id].end_time = end_time;
  SetPosture(PostureFrom(params));
  locomotion_.v_x = Clamp(v_x, limits.min_x, limits.max_x);
  locomotion_.v_y = Clamp(v_y, limits.min_y, limits.max_y);
  locomotion_.v_rot = Clamp(request.velocity().angular(), limits.min_rot, limits.max_rot);
  if (request.has_slew_rate_limit()) {
    // Non-positive values mean "robot default".
    const auto& slew = request.slew_rate_limit();
    if (slew.linear().x() > 0.0) locomotion_.max_accel_x = std::min(locomotion_.max_accel_x, slew.linear().x());
    if (slew.linear().y() > 0.0) locomotion_.max_accel_y = std::min(locomotion_.max_accel_y, slew.linear().y());
    if (slew.angular() > 0.0) locomotion_.max_accel_rot = std::min(locomotion_.max_accel_rot, slew.angular());
  }
  response.set_robot_command_id(id);
  response.set_status(api::RobotCommandResponse::STATUS_OK);
  return response;
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
    case Kind::kVelocity: {
      auto* mobility = feedback->mutable_synchronized_feedback()->mutable_mobility_command_feedback();
      mobility->set_status(record.timed_out ? api::RobotCommandFeedbackStatus::STATUS_COMMAND_TIMED_OUT : processing);
      mobility->mutable_se2_velocity_feedback();
      break;
    }
    case Kind::kFullBodyStop: {
      auto* full_body = feedback->mutable_full_body_feedback();
      full_body->set_status(processing);
      full_body->mutable_stop_feedback();
      break;
    }
    case Kind::kSelfRight: {
      auto* full_body = feedback->mutable_full_body_feedback();
      full_body->set_status(processing);
      full_body->mutable_selfright_feedback()->set_status(Reached(record, status, LocomotionState::kSitting)
                                                             ? api::SelfRightCommand::Feedback::STATUS_COMPLETED
                                                             : api::SelfRightCommand::Feedback::STATUS_IN_PROGRESS);
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
  // Velocity commands end at their end time; the robot then stands still.
  if (active_) {
    Record& record = records_[*active_];
    if (record.kind == Kind::kVelocity && now >= record.end_time) {
      record.timed_out = true;
      active_.reset();
      locomotion_.mode = LocomotionMode::kStand;
      locomotion_.v_x = locomotion_.v_y = locomotion_.v_rot = 0.0;
    }
  }

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
