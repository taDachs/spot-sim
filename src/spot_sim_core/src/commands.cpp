#include "spot_sim_core/commands.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>

#include <bosdyn/api/spot/robot_command.pb.h>

#include "spot_sim_core/arm_kinematics.hpp"
#include "spot_sim_core/robot_frames.hpp"
#include "spot_sim_core/stance.hpp"

namespace spot_sim {

namespace api = bosdyn::api;

namespace {
constexpr std::size_t kMaxRememberedCommands = 1024;
constexpr double kStowedTolerance = 0.05;          ///< [rad], as ManipulatorState.stow_state
constexpr double kDefaultGripperSpeed = 3.0;       ///< [rad/s]

/// std::visit with one lambda per alternative.
template <typename... Fs>
struct Overloaded : Fs... {
  using Fs::operator()...;
};
template <typename... Fs>
Overloaded(Fs...) -> Overloaded<Fs...>;

Rejection MotorsOff() { return Rejection{api::RobotCommandResponse::STATUS_NOT_POWERED_ON, "motors are off"}; }

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

CommandSystem::BodyPart* CommandSystem::ActiveBody() {
  Record* record = Find(active_);
  return record && record->body ? &*record->body : nullptr;
}

void CommandSystem::OverrideActive() {
  if (BodyPart* body = ActiveBody()) {
    body->overridden = true;
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

void CommandSystem::StartArm(uint32_t id, std::unique_ptr<ArmMotion> motion) {
  OverrideArm();
  arm_motion_ = std::move(motion);
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

uint32_t CommandSystem::Issue(BodyPart::Kind kind, LocomotionMode mode) {
  OverrideActive();
  const LocomotionCommand defaults;
  locomotion_.mode = mode;
  locomotion_.v_x = locomotion_.v_y = locomotion_.v_rot = 0.0;
  locomotion_.max_accel_x = defaults.max_accel_x;
  locomotion_.max_accel_y = defaults.max_accel_y;
  locomotion_.max_accel_rot = defaults.max_accel_rot;
  ++locomotion_.sequence;
  const uint32_t id = NewRecord();
  records_[id].body = BodyPart{std::move(kind), locomotion_.sequence};
  active_ = id;
  if (mode == LocomotionMode::kSit) {
    StowArm();  // the robot stows its arm before sitting down
  }
  return id;
}

api::RobotCommandResponse CommandSystem::Command(const api::RobotCommandRequest& request,
                                                 const api::LeaseUseResult& lease, const CommandContext& context) {
  api::RobotCommandResponse response;
  *response.mutable_lease_use_result() = lease;
  if (lease.status() != api::LeaseUseResult::STATUS_OK) {
    return response;
  }
  const auto& command = request.command();
  const Issued issued =
      command.has_full_body_command()      ? FullBody(command.full_body_command(), context)
      : command.has_synchronized_command() ? Synchronized(command.synchronized_command(), context)
                                           : Issued{Rejection{api::RobotCommandResponse::STATUS_INVALID_REQUEST,
                                                              "empty command"}};
  if (const auto* rejection = std::get_if<Rejection>(&issued)) {
    response.set_status(rejection->status);
    response.set_message(rejection->message);
  } else {
    response.set_robot_command_id(std::get<uint32_t>(issued));
    response.set_status(api::RobotCommandResponse::STATUS_OK);
  }
  return response;
}

CommandSystem::Issued CommandSystem::FullBody(const api::FullBodyCommand::Request& request,
                                              const CommandContext& context) {
  if (request.has_stop_request()) {
    HoldArm();
    HoldGripper();
    return Issue(BodyPart::FullBodyStop{}, LocomotionMode::kStop);
  }
  if (request.has_safe_power_off_request()) {
    // Done already if the motors are off.
    return Issue(BodyPart::SafePowerOff{!context.motors_on}, LocomotionMode::kSit);
  }
  if (request.has_selfright_request()) {
    // The simulated robot never falls over; self-righting ends lying down,
    // i.e. sitting.
    if (!context.motors_on) {
      return MotorsOff();
    }
    const uint32_t id = Issue(BodyPart::SelfRight{}, LocomotionMode::kSit);
    SetPosture({});
    return id;
  }
  return Rejection{api::RobotCommandResponse::STATUS_UNSUPPORTED, "full body command not simulated"};
}

CommandSystem::Issued CommandSystem::Synchronized(const api::SynchronizedCommand::Request& request,
                                                  const CommandContext& context) {
  const bool arm_part = request.has_arm_command();
  const bool gripper_part = request.has_gripper_command();
  if ((arm_part || gripper_part) && !has_arm_) {
    return Rejection{api::RobotCommandResponse::STATUS_UNSUPPORTED, "this robot has no arm"};
  }
  if (!request.has_mobility_command() && !arm_part && !gripper_part) {
    return Rejection{api::RobotCommandResponse::STATUS_INVALID_REQUEST, "empty synchronized command"};
  }
  // Check every part before executing any.
  std::unique_ptr<ArmMotion> arm_motion;
  if (arm_part) {
    auto created = ArmMotion::Create(request.arm_command(), CurrentArmContext(), context.now, context.clock_synced);
    if (auto* rejection = std::get_if<Rejection>(&created)) {
      return std::move(*rejection);
    }
    arm_motion = std::move(std::get<std::unique_ptr<ArmMotion>>(created));
  }
  if (gripper_part) {
    if (auto rejection = CheckGripper(request.gripper_command())) {
      return std::move(*rejection);
    }
  }
  if ((arm_part || gripper_part) && !context.motors_on) {
    return MotorsOff();
  }

  uint32_t id = 0;
  if (request.has_mobility_command()) {
    Issued mobility = Mobility(request.mobility_command(), context);
    if (std::holds_alternative<Rejection>(mobility)) {
      return mobility;
    }
    id = std::get<uint32_t>(mobility);
  } else {
    id = NewRecord();
  }
  if (arm_motion) {
    StartArm(id, std::move(arm_motion));
  }
  if (gripper_part) {
    StartGripper(id, request.gripper_command());
  }
  return id;
}

CommandSystem::Issued CommandSystem::Mobility(const api::MobilityCommand::Request& request,
                                              const CommandContext& context) {
  if (request.has_stop_request()) {
    return Issue(BodyPart::MobilityStop{}, LocomotionMode::kStop);
  }
  if (!context.motors_on) {
    return MotorsOff();
  }
  api::spot::MobilityParams params;
  if (request.has_params() && !request.params().UnpackTo(&params)) {
    return Rejection{api::RobotCommandResponse::STATUS_INVALID_REQUEST, "params is not a spot.MobilityParams"};
  }
  // Locomotion hints and stairs mode are accepted; the kinematic base walks
  // the same way regardless.
  if (request.has_stand_request()) {
    const uint32_t id = Issue(BodyPart::Stand{}, LocomotionMode::kStand);
    SetPosture(PostureFrom(params));
    return id;
  }
  if (request.has_sit_request()) {
    const uint32_t id = Issue(BodyPart::Sit{}, LocomotionMode::kSit);
    SetPosture({});
    return id;
  }
  if (request.has_se2_velocity_request()) {
    return Velocity(request.se2_velocity_request(), params, context);
  }
  return Rejection{api::RobotCommandResponse::STATUS_UNSUPPORTED, "mobility command not simulated yet"};
}

std::optional<Rejection> CommandSystem::CheckGripper(const api::GripperCommand::Request& request) {
  if (!request.has_claw_gripper_command()) {
    return Rejection{api::RobotCommandResponse::STATUS_INVALID_REQUEST, "gripper command not simulated"};
  }
  if (request.claw_gripper_command().trajectory().points().empty()) {
    return Rejection{api::RobotCommandResponse::STATUS_INVALID_REQUEST, "claw_gripper_command needs a trajectory point"};
  }
  return std::nullopt;
}

void CommandSystem::StartGripper(uint32_t id, const api::GripperCommand::Request& request) {
  OverrideGripper();
  const auto& claw = request.claw_gripper_command();
  // The trajectory's final angle; the gripper moves there at its speed limit.
  arm_.gripper = ArmKinematics::ClampedGripper(claw.trajectory().points().rbegin()->point());
  arm_.max_gripper_speed =
      std::min(kDefaultGripperSpeed, PositiveOr(claw.maximum_open_close_velocity(), kDefaultGripperSpeed));
  records_[id].gripper = GripperPart{};
  active_gripper_ = id;
}

void CommandSystem::SetPosture(const BodyPosture& posture) {
  locomotion_.body_height_offset = posture.height_offset;
  locomotion_.body_roll = posture.roll;
  locomotion_.body_pitch = posture.pitch;
  locomotion_.body_yaw = posture.yaw;
}

CommandSystem::Issued CommandSystem::Velocity(const api::SE2VelocityCommand::Request& request,
                                              const api::spot::MobilityParams& params, const CommandContext& context) {
  const auto end_time =
      CheckEndTime("se2_velocity_request", request.has_end_time(), request.end_time(), context.clock_synced, context.now);
  if (const auto* rejection = std::get_if<Rejection>(&end_time)) {
    return *rejection;
  }
  const auto frame = ParsePlanarFrame(request.se2_frame_name());
  if (!frame) {
    return Rejection{api::RobotCommandResponse::STATUS_UNKNOWN_FRAME,
                     std::string("se2_frame_name must be ") + kPlanarFrameNames};
  }

  // Express the velocity in the gravity-aligned body frame.
  double v_x = request.velocity().linear().x();
  double v_y = request.velocity().linear().y();
  if (*frame == RobotFrame::kOdom) {
    const double c = std::cos(context.odom_yaw), s = std::sin(context.odom_yaw);
    const double x = c * v_x + s * v_y;
    v_y = -s * v_x + c * v_y;
    v_x = x;
  }

  const VelocityLimits limits = LimitsFrom(params);

  const uint32_t id = Issue(BodyPart::Velocity{std::get<RobotTime>(end_time)}, LocomotionMode::kVelocity);
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
  return id;
}

bool CommandSystem::Reached(uint32_t sequence, const LocomotionStatus& status, LocomotionState state) {
  return status.state == state && status.reached_sequence >= sequence;
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

  auto* feedback = response.mutable_feedback();
  if (record.body) {
    const BodyPart& body = *record.body;
    const auto processing = processing_unless(body.overridden);
    const auto reached = [&](LocomotionState state) { return Reached(body.sequence, status, state); };
    const auto mobility = [&] {
      auto* mobility = feedback->mutable_synchronized_feedback()->mutable_mobility_command_feedback();
      mobility->set_status(processing);
      return mobility;
    };
    const auto full_body = [&] {
      auto* full_body = feedback->mutable_full_body_feedback();
      full_body->set_status(processing);
      return full_body;
    };
    std::visit(
        Overloaded{
            [&](const BodyPart::Stand&) {
              auto* stand = mobility()->mutable_stand_feedback();
              stand->set_status(reached(LocomotionState::kStanding) ? api::StandCommand::Feedback::STATUS_IS_STANDING
                                                                     : api::StandCommand::Feedback::STATUS_IN_PROGRESS);
              stand->set_standing_state(api::StandCommand::Feedback::STANDING_CONTROLLED);
            },
            [&](const BodyPart::Sit&) {
              mobility()->mutable_sit_feedback()->set_status(reached(LocomotionState::kSitting) && ArmStowed()
                                                                 ? api::SitCommand::Feedback::STATUS_IS_SITTING
                                                                 : api::SitCommand::Feedback::STATUS_IN_PROGRESS);
            },
            [&](const BodyPart::MobilityStop&) { mobility()->mutable_stop_feedback(); },
            [&](const BodyPart::Velocity& velocity) {
              auto* out = mobility();
              if (velocity.timed_out) {
                out->set_status(api::RobotCommandFeedbackStatus::STATUS_COMMAND_TIMED_OUT);
              }
              out->mutable_se2_velocity_feedback();
            },
            [&](const BodyPart::FullBodyStop&) { full_body()->mutable_stop_feedback(); },
            [&](const BodyPart::SelfRight&) {
              full_body()->mutable_selfright_feedback()->set_status(
                  reached(LocomotionState::kSitting) ? api::SelfRightCommand::Feedback::STATUS_COMPLETED
                                                     : api::SelfRightCommand::Feedback::STATUS_IN_PROGRESS);
            },
            [&](const BodyPart::SafePowerOff& off) {
              auto* out = full_body();
              if (off.done) {
                out->set_status(api::RobotCommandFeedbackStatus::STATUS_PROCESSING);
              }
              out->mutable_safe_power_off_feedback()->set_status(
                  off.done ? api::SafePowerOffCommand::Feedback::STATUS_POWERED_OFF
                           : api::SafePowerOffCommand::Feedback::STATUS_IN_PROGRESS);
            },
        },
        body.kind);
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
  if (BodyPart* body = ActiveBody()) {
    if (auto* velocity = std::get_if<BodyPart::Velocity>(&body->kind); velocity && now >= velocity->end_time) {
      velocity->timed_out = true;
      active_.reset();
      locomotion_.mode = LocomotionMode::kStand;
      locomotion_.v_x = locomotion_.v_y = locomotion_.v_rot = 0.0;
    }
  }

  // Safe power off: once sitting (arm stowed), cut motor power.
  if (BodyPart* body = ActiveBody()) {
    if (auto* off = std::get_if<BodyPart::SafePowerOff>(&body->kind);
        off && !off->done && power.MotorsOn() && Reached(body->sequence, status, LocomotionState::kSitting) &&
        ArmStowed()) {
      power.Cut();
      off->done = true;
    }
  }

  // SETTLE_THEN_CUT E-Stop: sit down, then cut (or cut after a timeout).
  if (estop_level == api::ESTOP_LEVEL_SETTLE_THEN_CUT && power.MotorsOn()) {
    if (!settling_since_) {
      settling_since_ = now;
      Issue(BodyPart::Sit{}, LocomotionMode::kSit);
      settle_sequence_ = locomotion_.sequence;
    }
    const bool seated = Reached(settle_sequence_, status, LocomotionState::kSitting) && ArmStowed();
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
    BodyPart* body = ActiveBody();
    if (auto* off = body ? std::get_if<BodyPart::SafePowerOff>(&body->kind) : nullptr) {
      // Power went away by other means (E-Stop); the robot is off anyway.
      off->done = true;
    } else {
      OverrideActive();
    }
    locomotion_.mode = LocomotionMode::kSit;
    if (has_arm_) {
      if (!arm_motion_ || !arm_motion_->is_hold() || active_arm_) {
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
