#include "spot_sim_core/commands.hpp"

#include <algorithm>
#include <cmath>

#include <bosdyn/api/spot/robot_command.pb.h>

#include "spot_sim_core/arm_kinematics.hpp"
#include "spot_sim_core/stance.hpp"

namespace spot_sim {

namespace api = bosdyn::api;

namespace {
constexpr std::size_t kMaxRememberedCommands = 1024;
constexpr double kArmReachedTolerance = 0.01;      ///< [rad]
constexpr double kStowedTolerance = 0.05;          ///< [rad], as ManipulatorState.stow_state
constexpr double kDefaultGripperSpeed = 3.0;       ///< [rad/s]

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

CommandSystem::CommandSystem(std::chrono::nanoseconds settle_timeout) : settle_timeout_(settle_timeout) {
  arm_.joints = ArmKinematics::Stow();
  arm_.gripper = ArmKinematics::kGripperClosed;
}

CommandSystem::Record* CommandSystem::Find(std::optional<uint32_t> id) {
  if (!id) {
    return nullptr;
  }
  const auto it = records_.find(*id);
  return it == records_.end() ? nullptr : &it->second;
}

void CommandSystem::OverrideActive() {
  if (Record* record = Find(active_)) {
    record->overridden = true;
  }
  active_.reset();
}

void CommandSystem::OverrideArm() {
  if (Record* record = Find(active_arm_); record && record->arm) {
    record->arm->overridden = true;
  }
  active_arm_.reset();
}

void CommandSystem::OverrideGripper() {
  if (Record* record = Find(active_gripper_); record && record->gripper) {
    record->gripper->overridden = true;
  }
  active_gripper_.reset();
}

void CommandSystem::HoldArm() {
  StartArm(0, ArmMotion::Hold(measured_arm_ ? measured_arm_->joints : arm_.joints));
}

void CommandSystem::HoldGripper() {
  OverrideGripper();
  if (measured_arm_) {
    arm_.gripper = measured_arm_->gripper;
  }
}

void CommandSystem::StowArm() {
  if (!has_arm_ ||
      (arm_motion_ && arm_motion_->named_position() == api::NamedArmPositionsCommand::POSITIONS_STOW)) {
    return;
  }
  StartArm(0, ArmMotion::ToJoints(ArmKinematics::Stow(), api::NamedArmPositionsCommand::POSITIONS_STOW));
}

ArmContext CommandSystem::CurrentArmContext() const {
  return ArmContext{measured_arm_ ? measured_arm_->joints : arm_.joints, odom_T_body_};
}

void CommandSystem::StartArm(uint32_t id, ArmMotion motion) {
  OverrideArm();
  arm_motion_ = std::make_shared<ArmMotion>(std::move(motion));
  if (id != 0) {
    records_[id].arm = ArmPart{arm_motion_};
    active_arm_ = id;
  }
  ApplyArm();
}

void CommandSystem::ApplyArm() {
  if (arm_motion_) {
    arm_.joints = arm_motion_->setpoint();
    arm_.max_joint_speed = arm_motion_->max_joint_speed();
  }
}

bool CommandSystem::ArmStowed() const {
  return !has_arm_ || (measured_arm_ && measured_arm_->joints.MaxDifference(ArmKinematics::Stow()) < kStowedTolerance);
}

uint32_t CommandSystem::NewRecord() {
  const uint32_t id = next_id_++;
  records_[id] = Record{};
  while (records_.size() > kMaxRememberedCommands) {
    records_.erase(records_.begin());
  }
  return id;
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
  const uint32_t id = NewRecord();
  Record& record = records_[id];
  record.has_body = true;
  record.kind = kind;
  record.sequence = locomotion_.sequence;
  active_ = id;
  if (mode == LocomotionMode::kSit) {
    StowArm();  // the robot stows its arm before sitting down
  }
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
      HoldArm();
      HoldGripper();
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
    const bool arm_part = sync.has_arm_command();
    const bool gripper_part = sync.has_gripper_command();
    if ((arm_part || gripper_part) && !has_arm_) {
      return reject(api::RobotCommandResponse::STATUS_UNSUPPORTED, "this robot has no arm");
    }
    if (!sync.has_mobility_command() && !arm_part && !gripper_part) {
      return reject(api::RobotCommandResponse::STATUS_INVALID_REQUEST, "empty synchronized command");
    }
    // Check every part before executing any.
    std::optional<ArmMotion> arm_motion;
    if (arm_part) {
      auto created = ArmMotion::Create(sync.arm_command(), CurrentArmContext(), now, clock_synced);
      if (const auto* rejection = std::get_if<ArmMotion::Rejection>(&created)) {
        return reject(rejection->status, rejection->message.c_str());
      }
      arm_motion = std::move(std::get<ArmMotion>(created));
    }
    if (gripper_part) {
      if (const auto error = CheckGripper(sync.gripper_command())) {
        return reject(api::RobotCommandResponse::STATUS_INVALID_REQUEST, error->c_str());
      }
    }
    if ((arm_part || gripper_part) && !motors_on) {
      return reject(api::RobotCommandResponse::STATUS_NOT_POWERED_ON, "motors are off");
    }

    uint32_t id = 0;
    if (sync.has_mobility_command()) {
      response = Mobility(sync.mobility_command(), motors_on, clock_synced, now, odom_yaw, std::move(response));
      if (response.status() != api::RobotCommandResponse::STATUS_OK) {
        return response;
      }
      id = response.robot_command_id();
    } else {
      id = NewRecord();
      response.set_robot_command_id(id);
      response.set_status(api::RobotCommandResponse::STATUS_OK);
    }
    if (arm_motion) {
      StartArm(id, std::move(*arm_motion));
    }
    if (gripper_part) {
      StartGripper(id, sync.gripper_command());
    }
    return response;
  }

  return reject(api::RobotCommandResponse::STATUS_INVALID_REQUEST, "empty command");
}

api::RobotCommandResponse CommandSystem::Mobility(const api::MobilityCommand::Request& mobility, bool motors_on,
                                                  bool clock_synced, RobotTime now, double odom_yaw,
                                                  api::RobotCommandResponse response) {
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

std::optional<std::string> CommandSystem::CheckGripper(const api::GripperCommand::Request& request) {
  if (!request.has_claw_gripper_command()) {
    return "gripper command not simulated";
  }
  if (request.claw_gripper_command().trajectory().points().empty()) {
    return "claw_gripper_command needs a trajectory point";
  }
  return std::nullopt;
}

void CommandSystem::StartGripper(uint32_t id, const api::GripperCommand::Request& request) {
  OverrideGripper();
  const auto& claw = request.claw_gripper_command();
  // The trajectory's final angle; the gripper moves there at its speed limit.
  arm_.gripper = ArmKinematics::ClampedGripper(claw.trajectory().points().rbegin()->point());
  arm_.max_gripper_speed = claw.has_maximum_open_close_velocity() && claw.maximum_open_close_velocity().value() > 0.0
                               ? std::min(kDefaultGripperSpeed, claw.maximum_open_close_velocity().value())
                               : kDefaultGripperSpeed;
  records_[id].gripper = GripperPart{};
  active_gripper_ = id;
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
  const auto processing_unless = [](bool overridden) {
    return overridden ? api::RobotCommandFeedbackStatus::STATUS_COMMAND_OVERRIDDEN
                      : api::RobotCommandFeedbackStatus::STATUS_PROCESSING;
  };
  const auto processing = processing_unless(record.overridden);

  auto* feedback = response.mutable_feedback();
  if (record.has_body) {
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
        mobility->mutable_sit_feedback()->set_status(Reached(record, status, LocomotionState::kSitting) && ArmStowed()
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
  }
  if (record.arm) {
    auto* arm = feedback->mutable_synchronized_feedback()->mutable_arm_command_feedback();
    const bool timed_out = record.arm->motion->progress() == ArmMotion::Progress::kTimedOut;
    arm->set_status(record.arm->overridden ? api::RobotCommandFeedbackStatus::STATUS_COMMAND_OVERRIDDEN
                    : timed_out            ? api::RobotCommandFeedbackStatus::STATUS_COMMAND_TIMED_OUT
                                           : api::RobotCommandFeedbackStatus::STATUS_PROCESSING);
    record.arm->motion->FillFeedback(arm);
  }
  if (record.gripper) {
    auto* gripper = feedback->mutable_synchronized_feedback()->mutable_gripper_command_feedback();
    gripper->set_status(processing_unless(record.gripper->overridden));
    gripper->mutable_claw_gripper_feedback()->set_status(record.gripper->reached
                                                            ? api::ClawGripperCommand::Feedback::STATUS_AT_GOAL
                                                            : api::ClawGripperCommand::Feedback::STATUS_IN_PROGRESS);
  }
  return response;
}

void CommandSystem::Step(RobotTime now, api::EstopStopLevel estop_level, PowerSystem& power,
                         const LocomotionStatus& status, const std::optional<ArmSample>& arm,
                         const Pose& odom_T_body) {
  if (arm) {
    measured_arm_ = arm;
  }
  odom_T_body_ = odom_T_body;

  // Velocity commands end at their end time; the robot then stands still.
  if (Record* record = Find(active_); record && record->kind == Kind::kVelocity && now >= record->end_time) {
    record->timed_out = true;
    active_.reset();
    locomotion_.mode = LocomotionMode::kStand;
    locomotion_.v_x = locomotion_.v_y = locomotion_.v_rot = 0.0;
  }

  // Safe power off: once sitting (arm stowed), cut motor power.
  if (Record* record = Find(active_); record && record->kind == Kind::kSafePowerOff && !record->done &&
                                      power.MotorsOn() && Reached(*record, status, LocomotionState::kSitting) &&
                                      ArmStowed()) {
    power.Cut();
    record->done = true;
  }

  // SETTLE_THEN_CUT E-Stop: sit down, then cut (or cut after a timeout).
  if (estop_level == api::ESTOP_LEVEL_SETTLE_THEN_CUT && power.MotorsOn()) {
    if (!settling_since_) {
      settling_since_ = now;
      Issue(Kind::kSit, LocomotionMode::kSit);
      settle_sequence_ = locomotion_.sequence;
    }
    const bool seated =
        status.state == LocomotionState::kSitting && status.reached_sequence >= settle_sequence_ && ArmStowed();
    if (seated || now - *settling_since_ >= settle_timeout_) {
      power.Cut(bosdyn::api::STATUS_ESTOPPED);
    }
  }
  if (estop_level == api::ESTOP_LEVEL_NONE || !power.MotorsOn()) {
    settling_since_.reset();
  }

  // Arm and gripper progress.
  if (measured_arm_) {
    if (arm_motion_) {
      arm_motion_->Step(now, CurrentArmContext());
      ApplyArm();
    }
    if (Record* record = Find(active_gripper_); record && record->gripper && !record->gripper->reached &&
                                                std::abs(measured_arm_->gripper - arm_.gripper) < kArmReachedTolerance) {
      record->gripper->reached = true;
    }
  }

  // Without motor power nothing is being executed any more; the arm stays
  // where it is.
  if (!power.MotorsOn()) {
    if (Record* record = Find(active_); record && record->kind == Kind::kSafePowerOff) {
      // Power went away by other means (E-Stop); the robot is off anyway.
      record->done = true;
    } else {
      OverrideActive();
    }
    locomotion_.mode = LocomotionMode::kSit;
    if (has_arm_) {
      if (!arm_motion_ || arm_motion_->kind() != ArmMotion::Kind::kStop || active_arm_) {
        HoldArm();
      }
      HoldGripper();
    }
  }
}

LocomotionCommand CommandSystem::Locomotion(bool motors_on) const {
  LocomotionCommand command = locomotion_;
  command.motors_on = motors_on;
  // Sit down only with the arm stowed; until then keep the current posture.
  if (motors_on && command.mode == LocomotionMode::kSit && !ArmStowed()) {
    command.mode = LocomotionMode::kStop;
  }
  return command;
}

ArmCommand CommandSystem::Arm(bool motors_on) const {
  ArmCommand command = arm_;
  command.motors_on = motors_on;
  return command;
}

}  // namespace spot_sim
