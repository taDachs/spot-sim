#include "spot_sim_core/arm_motion.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "spot_sim_core/arm_kinematics.hpp"

namespace spot_sim {

namespace api = bosdyn::api;

namespace {
constexpr double kMaxCartesianHandSpeed = 1.0;  ///< [m/s] for Cartesian velocity commands

double Seconds(RobotTime t) { return std::chrono::duration<double>(t).count(); }
double Seconds(const google::protobuf::Duration& d) { return std::chrono::duration<double>(FromDuration(d)).count(); }

Rejection UnknownFrame(const std::string& field) {
  return Rejection{api::RobotCommandResponse::STATUS_UNKNOWN_FRAME,
                   field + " must be " + kRobotFrameNames};
}

/// Joints of a trajectory point; joints it leaves out keep `previous`.
ArmJoints JointsOf(const api::ArmJointPosition& p, const ArmJoints& previous) {
  ArmJoints q = previous;
  if (p.has_sh0()) q.sh0 = p.sh0().value();
  if (p.has_sh1()) q.sh1 = p.sh1().value();
  if (p.has_el0()) q.el0 = p.el0().value();
  if (p.has_el1()) q.el1 = p.el1().value();
  if (p.has_wr0()) q.wr0 = p.wr0().value();
  if (p.has_wr1()) q.wr1 = p.wr1().value();
  return q;
}

ArmJoints Lerp(const ArmJoints& a, const ArmJoints& b, double s) {
  const auto x = a.ToArray(), y = b.ToArray();
  std::array<double, 6> out{};
  for (std::size_t i = 0; i < out.size(); ++i) {
    out[i] = x[i] + s * (y[i] - x[i]);
  }
  return ArmJoints::FromArray(out);
}

/// A trajectory point at robot time `t` [s].
template <typename Value>
struct Waypoint {
  double t;
  Value value;
};

/// Value of time-stamped waypoints at `t`, interpolated within its segment.
template <typename Value, typename Interpolate>
Value Sample(const std::vector<Waypoint<Value>>& waypoints, double t, Interpolate interpolate) {
  if (t <= waypoints.front().t) {
    return waypoints.front().value;
  }
  for (std::size_t i = 1; i < waypoints.size(); ++i) {
    if (t < waypoints[i].t) {
      const double span = waypoints[i].t - waypoints[i - 1].t;
      return interpolate(waypoints[i - 1].value, waypoints[i].value,
                         span > 0.0 ? (t - waypoints[i - 1].t) / span : 1.0);
    }
  }
  return waypoints.back().value;
}

/// Appends the points of a trajectory request to `waypoints` (which start
/// with where the arm is now, at `now_s`). Each point comes at its requested
/// time, but no sooner after the previous one than `min_duration(previous,
/// value)` allows. `value_of(point, previous)` gives a point's target.
/// Returns whether any point had to come later than requested.
template <typename Value, typename Trajectory, typename ValueOf, typename MinDuration>
bool AppendTimed(std::vector<Waypoint<Value>>& waypoints, const Trajectory& trajectory, double now_s,
                 ValueOf value_of, MinDuration min_duration) {
  const double reference =
      trajectory.has_reference_time() ? Seconds(FromTimestamp(trajectory.reference_time())) : now_s;
  bool stretched = false;
  for (const auto& point : trajectory.points()) {
    const Waypoint<Value> previous = waypoints.back();
    Value value = value_of(point, previous.value);
    const double earliest = previous.t + min_duration(previous.value, value);
    double t = earliest;
    if (point.has_time_since_reference()) {
      t = reference + Seconds(point.time_since_reference());
      if (t < earliest - 1e-9) {
        stretched = true;
        t = earliest;
      }
    }
    waypoints.push_back({t, std::move(value)});
  }
  return stretched;
}

/// Feedback status of a trajectory-like motion.
template <typename Status>
Status StatusOf(ArmMotion::Progress progress, Status complete, Status stalled, Status in_progress) {
  switch (progress) {
    case ArmMotion::Progress::kComplete:
      return complete;
    case ArmMotion::Progress::kStalled:
      return stalled;
    default:
      return in_progress;
  }
}

// ---- The kinds of motion -------------------------------------------------

/// Named positions and the internal stow: go to fixed joints.
class ToJointsMotion final : public ArmMotion {
 public:
  ToJointsMotion(const ArmJoints& target, std::optional<api::NamedArmPositionsCommand::Positions> named)
      : ArmMotion(ArmKinematics::Clamped(target), kJointSpeed), named_(named) {}

  void Step(RobotTime now, const ArmContext& context) override {
    const double now_s = Seconds(now);
    FinishWhenReached(now_s, now_s, context);
  }

  void FillFeedback(api::ArmCommand::Feedback* feedback) const override {
    feedback->mutable_named_arm_position_feedback()->set_status(
        progress_ == Progress::kComplete ? api::NamedArmPositionsCommand::Feedback::STATUS_COMPLETE
                                         : api::NamedArmPositionsCommand::Feedback::STATUS_IN_PROGRESS);
  }

  std::optional<api::NamedArmPositionsCommand::Positions> named_position() const override { return named_; }

 private:
  std::optional<api::NamedArmPositionsCommand::Positions> named_;
};

/// Arm stop: stay where the arm is.
class HoldMotion final : public ArmMotion {
 public:
  explicit HoldMotion(const ArmJoints& joints) : ArmMotion(joints, kJointSpeed) { progress_ = Progress::kComplete; }

  void Step(RobotTime, const ArmContext&) override {}
  void FillFeedback(api::ArmCommand::Feedback* feedback) const override { feedback->mutable_arm_stop_feedback(); }
  bool is_hold() const override { return true; }
};

class JointMoveMotion final : public ArmMotion {
 public:
  static std::variant<std::unique_ptr<ArmMotion>, Rejection> Create(const api::ArmJointMoveCommand::Request& request,
                                                                    const ArmContext& context, RobotTime now) {
    const auto& trajectory = request.trajectory();
    if (trajectory.points().empty()) {
      return Rejection{api::RobotCommandResponse::STATUS_INVALID_REQUEST, "arm joint trajectory has no points"};
    }
    auto motion = std::unique_ptr<JointMoveMotion>(new JointMoveMotion(context.joints));
    const double now_s = Seconds(now);
    const double speed = PositiveOr(trajectory.maximum_velocity(), kJointSpeed);
    bool clamped = false;
    motion->waypoints_.push_back({now_s, context.joints});
    const bool stretched = AppendTimed(
        motion->waypoints_, trajectory, now_s,
        [&](const api::ArmJointTrajectoryPoint& point, const ArmJoints& previous) {
          const ArmJoints wanted = JointsOf(point.position(), previous);
          const ArmJoints q = ArmKinematics::Clamped(wanted);
          clamped = clamped || q != wanted;
          return q;
        },
        [&](const ArmJoints& from, const ArmJoints& to) { return to.MaxDifference(from) / speed; });
    motion->modified_ = clamped || stretched;
    motion->end_s_ = motion->waypoints_.back().t;
    motion->now_s_ = now_s;
    return motion;
  }

  void Step(RobotTime now, const ArmContext& context) override {
    now_s_ = Seconds(now);
    setpoint_ = Sample(waypoints_, now_s_, Lerp);
    FinishWhenReached(now_s_, end_s_, context);
  }

  void FillFeedback(api::ArmCommand::Feedback* feedback) const override {
    auto* joint = feedback->mutable_arm_joint_move_feedback();
    joint->set_status(StatusOf(progress_, api::ArmJointMoveCommand::Feedback::STATUS_COMPLETE,
                               api::ArmJointMoveCommand::Feedback::STATUS_STALLED,
                               api::ArmJointMoveCommand::Feedback::STATUS_IN_PROGRESS));
    joint->set_planner_status(modified_ ? api::ArmJointMoveCommand::Feedback::PLANNER_STATUS_MODIFIED
                                        : api::ArmJointMoveCommand::Feedback::PLANNER_STATUS_SUCCESS);
    *joint->mutable_time_to_goal() = ToDuration(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(std::max(0.0, end_s_ - now_s_))));
  }

 private:
  explicit JointMoveMotion(const ArmJoints& joints) : ArmMotion(joints, kTrackingSpeed) {}

  std::vector<Waypoint<ArmJoints>> waypoints_;
  bool modified_ = false;  ///< the planner had to stretch the timing or clamp joints
  double end_s_ = 0.0;     ///< when the trajectory ends [s robot time]
  double now_s_ = 0.0;     ///< latest step [s robot time]
};

class CartesianMotion final : public ArmMotion {
 public:
  static std::variant<std::unique_ptr<ArmMotion>, Rejection> Create(const api::ArmCartesianCommand::Request& request,
                                                                    const ArmContext& context, RobotTime now) {
    const auto root = ParseRobotFrame(request.root_frame_name().empty() ? kBodyFrame : request.root_frame_name());
    if (!root) {
      return UnknownFrame("root_frame_name");
    }
    for (const auto mode : {request.x_axis(), request.y_axis(), request.z_axis(), request.rx_axis(),
                            request.ry_axis(), request.rz_axis()}) {
      if (mode == api::ArmCartesianCommand::Request::AXIS_MODE_FORCE) {
        return Rejection{api::RobotCommandResponse::STATUS_UNSUPPORTED, "force control is not simulated"};
      }
    }
    const auto& trajectory = request.pose_trajectory_in_task();
    if (trajectory.points().empty()) {
      return Rejection{api::RobotCommandResponse::STATUS_INVALID_REQUEST, "pose_trajectory_in_task has no points"};
    }

    auto motion = std::unique_ptr<CartesianMotion>(new CartesianMotion(context.joints));
    motion->root_ = *root;
    motion->root_T_task_ = request.has_root_tform_task() ? FromProto(request.root_tform_task()) : Pose{};
    motion->wrist_T_tool_ =
        request.has_wrist_tform_tool() ? FromProto(request.wrist_tform_tool()) : ArmKinematics::WristTHand();
    const double linear = PositiveOr(request.max_linear_velocity(), kLinearSpeed);
    const double angular = PositiveOr(request.max_angular_velocity(), kAngularSpeed);

    const double now_s = Seconds(now);
    const Pose task_T_tool =
        motion->BodyTTask(context).Inverse() * ArmKinematics::BodyTWrist(context.joints) * motion->wrist_T_tool_;
    motion->waypoints_.push_back({now_s, task_T_tool});
    // The Cartesian feedback has no planner status, so stretched timing goes
    // unreported.
    AppendTimed(
        motion->waypoints_, trajectory, now_s,
        [](const api::SE3TrajectoryPoint& point, const Pose&) { return FromProto(point.pose()); },
        [&](const Pose& from, const Pose& to) {
          return std::max((to.position - from.position).Norm() / linear, from.rotation.AngleTo(to.rotation) / angular);
        });
    motion->end_s_ = motion->waypoints_.back().t;
    return motion;
  }

  void Step(RobotTime now, const ArmContext& context) override {
    const double now_s = Seconds(now);
    const Pose body_T_task = BodyTTask(context);
    const Pose task_T_tool = Sample(waypoints_, now_s, [](const Pose& a, const Pose& b, double s) {
      return Pose{a.position + (b.position - a.position) * s, a.rotation.Slerp(b.rotation, s)};
    });
    SolveTool(body_T_task * task_T_tool, wrist_T_tool_);
    const Pose goal = body_T_task * waypoints_.back().value;
    const Pose tool = ArmKinematics::BodyTWrist(context.joints) * wrist_T_tool_;
    distance_to_goal_ = (goal.position - tool.position).Norm();
    angle_to_goal_ = goal.rotation.AngleTo(tool.rotation);
    FinishWhenReached(now_s, end_s_, context);
  }

  void FillFeedback(api::ArmCommand::Feedback* feedback) const override {
    auto* cartesian = feedback->mutable_arm_cartesian_feedback();
    cartesian->set_status(StatusOf(progress_, api::ArmCartesianCommand::Feedback::STATUS_TRAJECTORY_COMPLETE,
                                   api::ArmCartesianCommand::Feedback::STATUS_TRAJECTORY_STALLED,
                                   api::ArmCartesianCommand::Feedback::STATUS_IN_PROGRESS));
    cartesian->set_measured_pos_distance_to_goal(distance_to_goal_);
    cartesian->set_measured_rot_distance_to_goal(angle_to_goal_);
  }

 private:
  explicit CartesianMotion(const ArmJoints& joints) : ArmMotion(joints, kTrackingSpeed) {}

  Pose BodyTTask(const ArmContext& context) const { return BodyTFrame(root_, context.odom_T_body) * root_T_task_; }

  RobotFrame root_ = RobotFrame::kBody;
  Pose root_T_task_;
  Pose wrist_T_tool_;
  std::vector<Waypoint<Pose>> waypoints_;  ///< task_T_tool
  double end_s_ = 0.0;
  double distance_to_goal_ = 0.0;  ///< [m]
  double angle_to_goal_ = 0.0;     ///< [rad]
};

class VelocityMotion final : public ArmMotion {
 public:
  static std::variant<std::unique_ptr<ArmMotion>, Rejection> Create(const api::ArmVelocityCommand::Request& request,
                                                                    const ArmContext& context, RobotTime now,
                                                                    bool clock_synced) {
    const auto end = CheckEndTime("arm_velocity_command", request.has_end_time(), request.end_time(), clock_synced, now);
    if (const auto* rejection = std::get_if<Rejection>(&end)) {
      return *rejection;
    }
    auto motion = std::unique_ptr<VelocityMotion>(new VelocityMotion(context.joints));
    if (request.has_cylindrical_velocity()) {
      const auto& cylindrical = request.cylindrical_velocity();
      motion->cylindrical_ = true;
      motion->linear_ = Vec3{cylindrical.linear_velocity().r(), cylindrical.linear_velocity().theta(),
                             cylindrical.linear_velocity().z()};
      motion->max_linear_speed_ = PositiveOr(cylindrical.max_linear_velocity(), kLinearSpeed);
    } else if (request.has_cartesian_velocity()) {
      const auto& cartesian = request.cartesian_velocity();
      motion->cylindrical_ = false;
      if (cartesian.frame_name() != kHandFrame) {
        const auto frame = ParseRobotFrame(cartesian.frame_name());
        if (!frame) {
          return Rejection{api::RobotCommandResponse::STATUS_UNKNOWN_FRAME,
                           std::string("frame_name must be ") + kHandFrame + ", " + kRobotFrameNames};
        }
        motion->frame_ = frame;
      }
      motion->linear_ = FromProto(cartesian.velocity_in_frame_name());
      motion->max_linear_speed_ = kMaxCartesianHandSpeed;
    }
    motion->angular_in_hand_ = FromProto(request.angular_velocity_of_hand_rt_odom_in_hand());
    motion->body_T_hand_target_ = ArmKinematics::BodyTHand(context.joints);
    motion->last_s_ = Seconds(now);
    motion->end_s_ = Seconds(std::get<RobotTime>(end));
    return motion;
  }

  void Step(RobotTime now, const ArmContext& context) override {
    if (progress_ == Progress::kTimedOut) {
      return;
    }
    const double now_s = Seconds(now);
    const double t = std::min(now_s, end_s_);
    const double dt = t - last_s_;
    last_s_ = t;
    if (dt > 0.0) {
      Pose next = body_T_hand_target_;
      if (cylindrical_) {
        const Vec3 shoulder{ArmKinematics::kShoulderX, 0.0, ArmKinematics::kShoulderZ};
        const Vec3 p = next.position - shoulder;
        const double r = std::hypot(p.x, p.y), theta = std::atan2(p.y, p.x);
        Vec3 rate = linear_;  // (dr, dtheta, dz)
        const double speed = std::sqrt(rate.x * rate.x + rate.y * rate.y * r * r + rate.z * rate.z);
        if (speed > max_linear_speed_) {
          rate = rate * (max_linear_speed_ / speed);
        }
        const double r2 = std::max(0.0, r + rate.x * dt), theta2 = theta + rate.y * dt;
        next.position = shoulder + Vec3{r2 * std::cos(theta2), r2 * std::sin(theta2), p.z + rate.z * dt};
        next.rotation = Quat::FromYaw(theta2 - theta) * next.rotation;
      } else {
        const Quat body_R_frame = frame_ ? BodyTFrame(*frame_, context.odom_T_body).rotation : next.rotation;
        Vec3 v = body_R_frame.Rotate(linear_);
        if (v.Norm() > max_linear_speed_) {
          v = v * (max_linear_speed_ / v.Norm());
        }
        next.position = next.position + v * dt;
      }
      next.rotation = (next.rotation * Quat::FromRotationVector(angular_in_hand_ * dt)).Normalized();
      // At the edge of the workspace the hand stops where it is.
      if (const auto q = ArmKinematics::InverseHand(next, setpoint_)) {
        body_T_hand_target_ = next;
        setpoint_ = *q;
        reachable_ = true;
      } else {
        reachable_ = false;
      }
    }
    if (now_s >= end_s_) {
      progress_ = Progress::kTimedOut;
    }
  }

  void FillFeedback(api::ArmCommand::Feedback* feedback) const override { feedback->mutable_arm_velocity_feedback(); }

 private:
  explicit VelocityMotion(const ArmJoints& joints) : ArmMotion(joints, kTrackingSpeed) {}

  bool cylindrical_ = true;
  Vec3 linear_;                     ///< cylindrical (r, theta, z rates) or Cartesian in frame_
  std::optional<RobotFrame> frame_;  ///< Cartesian velocity frame; nullopt: the hand frame
  Vec3 angular_in_hand_;
  double max_linear_speed_ = kLinearSpeed;
  Pose body_T_hand_target_;
  double last_s_ = 0.0;  ///< latest integrated time [s robot time]
  double end_s_ = 0.0;   ///< end_time [s robot time]
};

class GazeMotion final : public ArmMotion {
 public:
  static std::variant<std::unique_ptr<ArmMotion>, Rejection> Create(const api::GazeCommand::Request& request,
                                                                    const ArmContext& context) {
    const auto target_frame = ParseRobotFrame(request.frame1_name());
    if (!target_frame) {
      return UnknownFrame("frame1_name");
    }
    if (request.target_trajectory_in_frame1().points().empty()) {
      return Rejection{api::RobotCommandResponse::STATUS_INVALID_REQUEST, "target_trajectory_in_frame1 has no points"};
    }
    auto motion = std::unique_ptr<GazeMotion>(new GazeMotion(context.joints));
    motion->target_frame_ = *target_frame;
    motion->target_ = FromProto(request.target_trajectory_in_frame1().points().rbegin()->point());
    motion->wrist_T_tool_ =
        request.has_wrist_tform_tool() ? FromProto(request.wrist_tform_tool()) : ArmKinematics::WristTHand();
    if (!request.tool_trajectory_in_frame2().points().empty()) {
      const auto tool_frame = ParseRobotFrame(request.frame2_name());
      if (!tool_frame) {
        return UnknownFrame("frame2_name");
      }
      motion->tool_frame_ = tool_frame;
      motion->tool_pose_ = FromProto(request.tool_trajectory_in_frame2().points().rbegin()->pose());
    } else {
      // The tool stays where it is, relative to the body.
      motion->tool_pose_ = ArmKinematics::BodyTWrist(context.joints) * motion->wrist_T_tool_;
    }
    return motion;
  }

  void Step(RobotTime now, const ArmContext& context) override {
    const Pose body_T_tool = tool_frame_ ? BodyTFrame(*tool_frame_, context.odom_T_body) * tool_pose_ : tool_pose_;
    const Vec3 target = (BodyTFrame(target_frame_, context.odom_T_body) * Pose{target_, Quat{}}).position;
    Quat rotation = body_T_tool.rotation;
    const Vec3 direction = target - body_T_tool.position;
    if (direction.Norm() > 1e-6) {
      // Tool x axis at the target, no roll: y horizontal (in odom).
      const Vec3 x = direction.Normalized();
      const Vec3 up = context.odom_T_body.rotation.Conjugate().Rotate(Vec3{0.0, 0.0, 1.0});
      Vec3 y = up.Cross(x);
      y = y.Norm() < 1e-6 ? Vec3{0.0, 1.0, 0.0} : y.Normalized();
      rotation = Quat::FromAxes(x, y, x.Cross(y));
    }
    SolveTool(Pose{body_T_tool.position, rotation}, wrist_T_tool_);
    const Pose tool = ArmKinematics::BodyTWrist(context.joints) * wrist_T_tool_;
    distance_to_goal_ = (body_T_tool.position - tool.position).Norm();
    const Vec3 to_target = target - tool.position;
    angle_to_goal_ = to_target.Norm() > 1e-6
                         ? std::acos(std::clamp(tool.rotation.XAxis().Dot(to_target.Normalized()), -1.0, 1.0))
                         : 0.0;
    const double now_s = Seconds(now);
    FinishWhenReached(now_s, now_s, context);
  }

  void FillFeedback(api::ArmCommand::Feedback* feedback) const override {
    auto* gaze = feedback->mutable_arm_gaze_feedback();
    gaze->set_status(StatusOf(progress_, api::GazeCommand::Feedback::STATUS_TRAJECTORY_COMPLETE,
                              api::GazeCommand::Feedback::STATUS_TOOL_TRAJECTORY_STALLED,
                              api::GazeCommand::Feedback::STATUS_IN_PROGRESS));
    constexpr double kAngleTolerance = 0.05, kDistanceTolerance = 0.01;
    gaze->set_gazing_at_target(angle_to_goal_ < kAngleTolerance);
    gaze->set_gaze_to_target_rotation_measured(static_cast<float>(angle_to_goal_));
    gaze->set_hand_position_at_goal(distance_to_goal_ < kDistanceTolerance);
    gaze->set_hand_distance_to_goal_measured(static_cast<float>(distance_to_goal_));
    gaze->set_hand_roll_at_goal(progress_ == Progress::kComplete);
  }

 private:
  explicit GazeMotion(const ArmJoints& joints) : ArmMotion(joints, kJointSpeed) {}

  RobotFrame target_frame_ = RobotFrame::kBody;
  Vec3 target_;  ///< in target_frame_
  std::optional<RobotFrame> tool_frame_;
  Pose tool_pose_;  ///< in tool_frame_, or body_T_tool when held
  Pose wrist_T_tool_;
  double distance_to_goal_ = 0.0;  ///< [m]
  double angle_to_goal_ = 0.0;     ///< [rad]
};
}  // namespace

std::unique_ptr<ArmMotion> ArmMotion::ToJoints(const ArmJoints& target, api::NamedArmPositionsCommand::Positions position) {
  return std::make_unique<ToJointsMotion>(
      target, position == api::NamedArmPositionsCommand::POSITIONS_UNKNOWN
                  ? std::nullopt
                  : std::optional<api::NamedArmPositionsCommand::Positions>(position));
}

std::unique_ptr<ArmMotion> ArmMotion::Hold(const ArmJoints& joints) { return std::make_unique<HoldMotion>(joints); }

std::variant<std::unique_ptr<ArmMotion>, Rejection> ArmMotion::Create(const api::ArmCommand::Request& request,
                                                                      const ArmContext& context, RobotTime now,
                                                                      bool clock_synced) {
  switch (request.command_case()) {
    case api::ArmCommand::Request::kNamedArmPositionCommand:
      switch (request.named_arm_position_command().position()) {
        case api::NamedArmPositionsCommand::POSITIONS_STOW:
          return ToJoints(ArmKinematics::Stow(), api::NamedArmPositionsCommand::POSITIONS_STOW);
        case api::NamedArmPositionsCommand::POSITIONS_READY:
          return ToJoints(ArmKinematics::Ready(), api::NamedArmPositionsCommand::POSITIONS_READY);
        case api::NamedArmPositionsCommand::POSITIONS_CARRY:
          return ToJoints(ArmKinematics::Carry(), api::NamedArmPositionsCommand::POSITIONS_CARRY);
        default:
          return Rejection{api::RobotCommandResponse::STATUS_INVALID_REQUEST,
                           "named_arm_position_command needs a position"};
      }
    case api::ArmCommand::Request::kArmStopCommand:
      return Hold(context.joints);
    case api::ArmCommand::Request::kArmJointMoveCommand:
      return JointMoveMotion::Create(request.arm_joint_move_command(), context, now);
    case api::ArmCommand::Request::kArmCartesianCommand:
      return CartesianMotion::Create(request.arm_cartesian_command(), context, now);
    case api::ArmCommand::Request::kArmVelocityCommand:
      return VelocityMotion::Create(request.arm_velocity_command(), context, now, clock_synced);
    case api::ArmCommand::Request::kArmGazeCommand:
      return GazeMotion::Create(request.arm_gaze_command(), context);
    default:
      return Rejection{api::RobotCommandResponse::STATUS_UNSUPPORTED, "arm command not simulated"};
  }
}

bool ArmMotion::SolveTool(const Pose& body_T_tool, const Pose& wrist_T_tool) {
  const auto q = ArmKinematics::InverseWrist(body_T_tool * wrist_T_tool.Inverse(), setpoint_);
  reachable_ = q.has_value();
  if (q) {
    setpoint_ = *q;
  }
  return reachable_;
}

void ArmMotion::FinishWhenReached(double now_s, double end_s, const ArmContext& context) {
  if (now_s < end_s || progress_ == Progress::kComplete) {
    return;
  }
  if (!reachable_) {
    progress_ = Progress::kStalled;
    return;
  }
  if (context.joints.MaxDifference(setpoint_) < kArmReachedTolerance) {
    progress_ = Progress::kComplete;
  }
}

}  // namespace spot_sim
