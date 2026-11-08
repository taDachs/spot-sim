#include "spot_sim_core/arm_motion.hpp"

#include <algorithm>
#include <cmath>

#include "spot_sim_core/arm_kinematics.hpp"

namespace spot_sim {

namespace api = bosdyn::api;

namespace {
constexpr double kReachedTolerance = 0.01;  ///< [rad]
constexpr double kMaxCartesianHandSpeed = 1.0;  ///< [m/s] for Cartesian velocity commands

double Seconds(RobotTime t) { return std::chrono::duration<double>(t).count(); }
double Seconds(const google::protobuf::Duration& d) { return std::chrono::duration<double>(FromDuration(d)).count(); }

ArmMotion::Rejection Reject(api::RobotCommandResponse::Status status, std::string message) {
  return ArmMotion::Rejection{status, std::move(message)};
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

/// Segment and fraction of `t` along time-stamped waypoints.
template <typename Waypoint, typename Interpolate>
auto Sample(const std::vector<Waypoint>& waypoints, double t, Interpolate interpolate) {
  if (t <= waypoints.front().t) {
    return interpolate(waypoints.front(), waypoints.front(), 0.0);
  }
  for (std::size_t i = 1; i < waypoints.size(); ++i) {
    if (t < waypoints[i].t) {
      const double span = waypoints[i].t - waypoints[i - 1].t;
      return interpolate(waypoints[i - 1], waypoints[i], span > 0.0 ? (t - waypoints[i - 1].t) / span : 1.0);
    }
  }
  return interpolate(waypoints.back(), waypoints.back(), 0.0);
}

/// Earliest arrival at the next waypoint given the speed limits.
double EarliestArrival(double from_t, const Pose& from, const Pose& to, double linear, double angular) {
  return from_t + std::max((to.position - from.position).Norm() / linear, from.rotation.AngleTo(to.rotation) / angular);
}
}  // namespace

std::optional<RobotFrame> ParseRobotFrame(const std::string& name) {
  if (name == "body") return RobotFrame::kBody;
  if (name == "flat_body") return RobotFrame::kFlatBody;
  if (name == "odom" || name == "vision" || name == "gpe") return RobotFrame::kOdom;
  return std::nullopt;
}

Pose BodyTFrame(RobotFrame frame, const Pose& odom_T_body) {
  switch (frame) {
    case RobotFrame::kBody:
      return Pose{};
    case RobotFrame::kFlatBody: {
      const Pose odom_T_flat{odom_T_body.position, Quat::FromYaw(odom_T_body.rotation.Yaw())};
      return odom_T_body.Inverse() * odom_T_flat;
    }
    case RobotFrame::kOdom:
      break;
  }
  return odom_T_body.Inverse();
}

ArmMotion ArmMotion::ToJoints(const ArmJoints& target, api::NamedArmPositionsCommand::Positions position) {
  ArmMotion motion(Kind::kNamed);
  motion.setpoint_ = ArmKinematics::Clamped(target);
  if (position != api::NamedArmPositionsCommand::POSITIONS_UNKNOWN) {
    motion.named_ = position;
  }
  return motion;
}

ArmMotion ArmMotion::Hold(const ArmJoints& joints) {
  ArmMotion motion(Kind::kStop);
  motion.setpoint_ = joints;
  motion.progress_ = Progress::kComplete;
  return motion;
}

std::variant<ArmMotion, ArmMotion::Rejection> ArmMotion::Create(const api::ArmCommand::Request& request,
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
          return Reject(api::RobotCommandResponse::STATUS_INVALID_REQUEST,
                        "named_arm_position_command needs a position");
      }
    case api::ArmCommand::Request::kArmStopCommand: {
      ArmMotion motion = Hold(context.joints);
      return motion;
    }
    case api::ArmCommand::Request::kArmJointMoveCommand:
      return JointMove(request.arm_joint_move_command(), context, now);
    case api::ArmCommand::Request::kArmCartesianCommand:
      return Cartesian(request.arm_cartesian_command(), context, now);
    case api::ArmCommand::Request::kArmVelocityCommand:
      return Velocity(request.arm_velocity_command(), context, now, clock_synced);
    case api::ArmCommand::Request::kArmGazeCommand:
      return Gaze(request.arm_gaze_command(), context, now);
    default:
      return Reject(api::RobotCommandResponse::STATUS_UNSUPPORTED, "arm command not simulated");
  }
}

std::variant<ArmMotion, ArmMotion::Rejection> ArmMotion::JointMove(const api::ArmJointMoveCommand::Request& request,
                                                                   const ArmContext& context, RobotTime now) {
  const auto& trajectory = request.trajectory();
  if (trajectory.points().empty()) {
    return Reject(api::RobotCommandResponse::STATUS_INVALID_REQUEST, "arm joint trajectory has no points");
  }
  ArmMotion motion(Kind::kJointMove);
  const double now_s = Seconds(now);
  const double reference = trajectory.has_reference_time() ? Seconds(FromTimestamp(trajectory.reference_time())) : now_s;
  const double speed = trajectory.has_maximum_velocity() && trajectory.maximum_velocity().value() > 0.0
                           ? trajectory.maximum_velocity().value()
                           : kJointSpeed;
  motion.joint_waypoints_.push_back({now_s, context.joints});
  for (const auto& point : trajectory.points()) {
    const auto& previous = motion.joint_waypoints_.back();
    const ArmJoints wanted = JointsOf(point.position(), previous.joints);
    const ArmJoints q = ArmKinematics::Clamped(wanted);
    if (q != wanted) {
      motion.modified_ = true;
    }
    const double earliest = previous.t + q.MaxDifference(previous.joints) / speed;
    double t = earliest;
    if (point.has_time_since_reference()) {
      t = reference + Seconds(point.time_since_reference());
      if (t < earliest - 1e-9) {
        motion.modified_ = true;
        t = earliest;
      }
    }
    motion.joint_waypoints_.push_back({t, q});
  }
  motion.end_s_ = motion.joint_waypoints_.back().t;
  motion.setpoint_ = context.joints;
  motion.max_joint_speed_ = kTrackingSpeed;
  motion.now_s_ = now_s;
  return motion;
}

std::variant<ArmMotion, ArmMotion::Rejection> ArmMotion::Cartesian(const api::ArmCartesianCommand::Request& request,
                                                                   const ArmContext& context, RobotTime now) {
  const std::string root_name = request.root_frame_name().empty() ? "body" : request.root_frame_name();
  const auto root = ParseRobotFrame(root_name);
  if (!root) {
    return Reject(api::RobotCommandResponse::STATUS_UNKNOWN_FRAME,
                  "root_frame_name must be body, flat_body, odom, vision or gpe");
  }
  for (const auto mode : {request.x_axis(), request.y_axis(), request.z_axis(), request.rx_axis(), request.ry_axis(),
                          request.rz_axis()}) {
    if (mode == api::ArmCartesianCommand::Request::AXIS_MODE_FORCE) {
      return Reject(api::RobotCommandResponse::STATUS_UNSUPPORTED, "force control is not simulated");
    }
  }
  const auto& trajectory = request.pose_trajectory_in_task();
  if (trajectory.points().empty()) {
    return Reject(api::RobotCommandResponse::STATUS_INVALID_REQUEST, "pose_trajectory_in_task has no points");
  }

  ArmMotion motion(Kind::kCartesian);
  motion.root_ = *root;
  motion.root_T_task_ = request.has_root_tform_task() ? FromProto(request.root_tform_task()) : Pose{};
  motion.wrist_T_tool_ = request.has_wrist_tform_tool() ? FromProto(request.wrist_tform_tool()) : ArmKinematics::WristTHand();
  const double linear = request.has_max_linear_velocity() && request.max_linear_velocity().value() > 0.0
                            ? request.max_linear_velocity().value()
                            : kLinearSpeed;
  const double angular = request.has_max_angular_velocity() && request.max_angular_velocity().value() > 0.0
                             ? request.max_angular_velocity().value()
                             : kAngularSpeed;

  const double now_s = Seconds(now);
  const double reference = trajectory.has_reference_time() ? Seconds(FromTimestamp(trajectory.reference_time())) : now_s;
  const Pose body_T_task = BodyTFrame(motion.root_, context.odom_T_body) * motion.root_T_task_;
  const Pose task_T_tool =
      body_T_task.Inverse() * ArmKinematics::BodyTWrist(context.joints) * motion.wrist_T_tool_;
  motion.pose_waypoints_.push_back({now_s, task_T_tool});
  for (const auto& point : trajectory.points()) {
    const auto& previous = motion.pose_waypoints_.back();
    const Pose pose = FromProto(point.pose());
    const double earliest = EarliestArrival(previous.t, previous.pose, pose, linear, angular);
    double t = earliest;
    if (point.has_time_since_reference()) {
      t = reference + Seconds(point.time_since_reference());
      if (t < earliest - 1e-9) {
        motion.modified_ = true;
        t = earliest;
      }
    }
    motion.pose_waypoints_.push_back({t, pose});
  }
  motion.end_s_ = motion.pose_waypoints_.back().t;
  motion.setpoint_ = context.joints;
  motion.max_joint_speed_ = kTrackingSpeed;
  motion.now_s_ = now_s;
  return motion;
}

std::variant<ArmMotion, ArmMotion::Rejection> ArmMotion::Velocity(const api::ArmVelocityCommand::Request& request,
                                                                  const ArmContext& context, RobotTime now,
                                                                  bool clock_synced) {
  if (!request.has_end_time()) {
    return Reject(api::RobotCommandResponse::STATUS_INVALID_REQUEST, "arm_velocity_command requires an end_time");
  }
  if (!clock_synced) {
    return Reject(api::RobotCommandResponse::STATUS_NO_TIMESYNC, "clock_identifier has no established time sync");
  }
  const RobotTime end = FromTimestamp(request.end_time());
  if (end <= now) {
    return Reject(api::RobotCommandResponse::STATUS_EXPIRED, "end_time has already passed");
  }
  ArmMotion motion(Kind::kVelocity);
  if (request.has_cylindrical_velocity()) {
    const auto& cylindrical = request.cylindrical_velocity();
    motion.cylindrical_ = true;
    motion.linear_ = Vec3{cylindrical.linear_velocity().r(), cylindrical.linear_velocity().theta(),
                          cylindrical.linear_velocity().z()};
    if (cylindrical.has_max_linear_velocity() && cylindrical.max_linear_velocity().value() > 0.0) {
      motion.max_linear_speed_ = cylindrical.max_linear_velocity().value();
    }
  } else if (request.has_cartesian_velocity()) {
    const auto& cartesian = request.cartesian_velocity();
    motion.cylindrical_ = false;
    if (cartesian.frame_name() != "hand") {
      const auto frame = ParseRobotFrame(cartesian.frame_name());
      if (!frame) {
        return Reject(api::RobotCommandResponse::STATUS_UNKNOWN_FRAME,
                      "frame_name must be body, flat_body, odom, vision, gpe or hand");
      }
      motion.velocity_frame_ = frame;
    }
    motion.linear_ = FromProto(cartesian.velocity_in_frame_name());
    motion.max_linear_speed_ = kMaxCartesianHandSpeed;
  }
  motion.angular_in_hand_ = FromProto(request.angular_velocity_of_hand_rt_odom_in_hand());
  motion.body_T_hand_target_ = ArmKinematics::BodyTHand(context.joints);
  motion.setpoint_ = context.joints;
  motion.max_joint_speed_ = kTrackingSpeed;
  motion.now_s_ = motion.last_s_ = Seconds(now);
  motion.end_s_ = Seconds(end);
  return motion;
}

std::variant<ArmMotion, ArmMotion::Rejection> ArmMotion::Gaze(const api::GazeCommand::Request& request,
                                                              const ArmContext& context, RobotTime now) {
  const auto target_frame = ParseRobotFrame(request.frame1_name());
  if (!target_frame) {
    return Reject(api::RobotCommandResponse::STATUS_UNKNOWN_FRAME, "frame1_name must be body, flat_body, odom, vision or gpe");
  }
  if (request.target_trajectory_in_frame1().points().empty()) {
    return Reject(api::RobotCommandResponse::STATUS_INVALID_REQUEST, "target_trajectory_in_frame1 has no points");
  }
  ArmMotion motion(Kind::kGaze);
  motion.target_frame_ = *target_frame;
  motion.target_ = FromProto(request.target_trajectory_in_frame1().points().rbegin()->point());
  motion.wrist_T_tool_ = request.has_wrist_tform_tool() ? FromProto(request.wrist_tform_tool()) : ArmKinematics::WristTHand();
  if (!request.tool_trajectory_in_frame2().points().empty()) {
    const auto tool_frame = ParseRobotFrame(request.frame2_name());
    if (!tool_frame) {
      return Reject(api::RobotCommandResponse::STATUS_UNKNOWN_FRAME,
                    "frame2_name must be body, flat_body, odom, vision or gpe");
    }
    motion.tool_frame_ = tool_frame;
    motion.tool_pose_ = FromProto(request.tool_trajectory_in_frame2().points().rbegin()->pose());
  } else {
    // The tool stays where it is, relative to the body.
    motion.tool_pose_ = ArmKinematics::BodyTWrist(context.joints) * motion.wrist_T_tool_;
  }
  motion.setpoint_ = context.joints;
  motion.now_s_ = Seconds(now);
  return motion;
}

bool ArmMotion::SolveTool(const Pose& body_T_tool, const ArmJoints& near) {
  const auto q = ArmKinematics::InverseWrist(body_T_tool * wrist_T_tool_.Inverse(), near);
  reachable_ = q.has_value();
  if (q) {
    setpoint_ = *q;
  }
  return reachable_;
}

void ArmMotion::FinishWhenReached(double now_s, const ArmContext& context) {
  if (now_s < end_s_ || progress_ == Progress::kComplete) {
    return;
  }
  if (!reachable_) {
    progress_ = Progress::kStalled;
    return;
  }
  if (context.joints.MaxDifference(setpoint_) < kReachedTolerance) {
    progress_ = Progress::kComplete;
  }
}

void ArmMotion::Step(RobotTime now, const ArmContext& context) {
  const double now_s = Seconds(now);
  now_s_ = now_s;
  switch (kind_) {
    case Kind::kNamed:
      FinishWhenReached(now_s, context);
      break;
    case Kind::kStop:
      break;
    case Kind::kJointMove:
      setpoint_ = Sample(joint_waypoints_, now_s, [](const JointWaypoint& a, const JointWaypoint& b, double s) {
        return Lerp(a.joints, b.joints, s);
      });
      FinishWhenReached(now_s, context);
      break;
    case Kind::kCartesian: {
      const Pose body_T_task = BodyTFrame(root_, context.odom_T_body) * root_T_task_;
      const Pose task_T_tool = Sample(pose_waypoints_, now_s, [](const PoseWaypoint& a, const PoseWaypoint& b, double s) {
        return Pose{a.pose.position + (b.pose.position - a.pose.position) * s, a.pose.rotation.Slerp(b.pose.rotation, s)};
      });
      SolveTool(body_T_task * task_T_tool, setpoint_);
      const Pose goal = body_T_task * pose_waypoints_.back().pose;
      const Pose tool = ArmKinematics::BodyTWrist(context.joints) * wrist_T_tool_;
      distance_to_goal_ = (goal.position - tool.position).Norm();
      angle_to_goal_ = goal.rotation.AngleTo(tool.rotation);
      FinishWhenReached(now_s, context);
      break;
    }
    case Kind::kVelocity: {
      if (progress_ == Progress::kTimedOut) {
        break;
      }
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
          const Quat body_R_frame =
              velocity_frame_ ? BodyTFrame(*velocity_frame_, context.odom_T_body).rotation : next.rotation;
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
      break;
    }
    case Kind::kGaze: {
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
      SolveTool(Pose{body_T_tool.position, rotation}, setpoint_);
      const Pose tool = ArmKinematics::BodyTWrist(context.joints) * wrist_T_tool_;
      distance_to_goal_ = (body_T_tool.position - tool.position).Norm();
      const Vec3 to_target = target - tool.position;
      angle_to_goal_ = to_target.Norm() > 1e-6
                           ? std::acos(std::clamp(tool.rotation.XAxis().Dot(to_target.Normalized()), -1.0, 1.0))
                           : 0.0;
      FinishWhenReached(now_s, context);
      break;
    }
  }
}

void ArmMotion::FillFeedback(api::ArmCommand::Feedback* feedback) const {
  switch (kind_) {
    case Kind::kNamed:
      feedback->mutable_named_arm_position_feedback()->set_status(
          progress_ == Progress::kComplete ? api::NamedArmPositionsCommand::Feedback::STATUS_COMPLETE
                                           : api::NamedArmPositionsCommand::Feedback::STATUS_IN_PROGRESS);
      break;
    case Kind::kStop:
      feedback->mutable_arm_stop_feedback();
      break;
    case Kind::kJointMove: {
      auto* joint = feedback->mutable_arm_joint_move_feedback();
      joint->set_status(progress_ == Progress::kComplete  ? api::ArmJointMoveCommand::Feedback::STATUS_COMPLETE
                        : progress_ == Progress::kStalled ? api::ArmJointMoveCommand::Feedback::STATUS_STALLED
                                                          : api::ArmJointMoveCommand::Feedback::STATUS_IN_PROGRESS);
      joint->set_planner_status(modified_ ? api::ArmJointMoveCommand::Feedback::PLANNER_STATUS_MODIFIED
                                          : api::ArmJointMoveCommand::Feedback::PLANNER_STATUS_SUCCESS);
      *joint->mutable_time_to_goal() =
          ToDuration(std::chrono::duration_cast<std::chrono::nanoseconds>(
              std::chrono::duration<double>(std::max(0.0, end_s_ - now_s_))));
      break;
    }
    case Kind::kCartesian: {
      auto* cartesian = feedback->mutable_arm_cartesian_feedback();
      cartesian->set_status(progress_ == Progress::kComplete
                                ? api::ArmCartesianCommand::Feedback::STATUS_TRAJECTORY_COMPLETE
                            : progress_ == Progress::kStalled
                                ? api::ArmCartesianCommand::Feedback::STATUS_TRAJECTORY_STALLED
                                : api::ArmCartesianCommand::Feedback::STATUS_IN_PROGRESS);
      cartesian->set_measured_pos_distance_to_goal(distance_to_goal_);
      cartesian->set_measured_rot_distance_to_goal(angle_to_goal_);
      break;
    }
    case Kind::kVelocity:
      feedback->mutable_arm_velocity_feedback();
      break;
    case Kind::kGaze: {
      auto* gaze = feedback->mutable_arm_gaze_feedback();
      gaze->set_status(progress_ == Progress::kComplete  ? api::GazeCommand::Feedback::STATUS_TRAJECTORY_COMPLETE
                       : progress_ == Progress::kStalled ? api::GazeCommand::Feedback::STATUS_TOOL_TRAJECTORY_STALLED
                                                         : api::GazeCommand::Feedback::STATUS_IN_PROGRESS);
      constexpr double kAngleTolerance = 0.05, kDistanceTolerance = 0.01;
      gaze->set_gazing_at_target(angle_to_goal_ < kAngleTolerance);
      gaze->set_gaze_to_target_rotation_measured(static_cast<float>(angle_to_goal_));
      gaze->set_hand_position_at_goal(distance_to_goal_ < kDistanceTolerance);
      gaze->set_hand_distance_to_goal_measured(static_cast<float>(distance_to_goal_));
      gaze->set_hand_roll_at_goal(progress_ == Progress::kComplete);
      break;
    }
  }
}

}  // namespace spot_sim
