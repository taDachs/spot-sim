// Arm inverse kinematics and ArmMotion: joint moves, Cartesian moves,
// velocity, gaze.

#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <variant>

#include "spot_sim_core/arm_kinematics.hpp"
#include "spot_sim_core/arm_motion.hpp"
#include "spot_sim_core/commands.hpp"

using namespace std::chrono_literals;
namespace api = bosdyn::api;
using spot_sim::ArmContext;
using spot_sim::ArmJoints;
using spot_sim::ArmKinematics;
using spot_sim::ArmMotion;
using spot_sim::Pose;
using spot_sim::Quat;
using spot_sim::Vec3;

namespace {
ArmMotion Create(const api::ArmCommand::Request& request, const ArmContext& context, spot_sim::RobotTime now = 0s,
                 bool clock_synced = true) {
  auto created = ArmMotion::Create(request, context, now, clock_synced);
  if (const auto* rejection = std::get_if<ArmMotion::Rejection>(&created)) {
    ADD_FAILURE() << "rejected: " << rejection->message;
    return ArmMotion::Hold(context.joints);
  }
  return std::get<ArmMotion>(created);
}

api::RobotCommandResponse::Status Rejection(const api::ArmCommand::Request& request, const ArmContext& context,
                                            spot_sim::RobotTime now = 0s, bool clock_synced = true) {
  auto created = ArmMotion::Create(request, context, now, clock_synced);
  const auto* rejection = std::get_if<ArmMotion::Rejection>(&created);
  return rejection ? rejection->status : api::RobotCommandResponse::STATUS_OK;
}

/// Steps `motion` from `from` to `to` with an ideal arm that is always at the setpoint.
void Simulate(ArmMotion& motion, ArmContext& context, double from, double to, double dt = 0.01) {
  for (double t = from; t <= to + 1e-9; t += dt) {
    motion.Step(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(t)), context);
    context.joints = motion.setpoint();
  }
}

api::ArmCommand::Request Cartesian(const std::string& root, const Pose& pose, double seconds = -1.0) {
  api::ArmCommand::Request request;
  auto* cartesian = request.mutable_arm_cartesian_command();
  cartesian->set_root_frame_name(root);
  auto* point = cartesian->mutable_pose_trajectory_in_task()->add_points();
  *point->mutable_pose() = spot_sim::ToProto(pose);
  if (seconds >= 0.0) {
    *point->mutable_time_since_reference() =
        spot_sim::ToDuration(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(seconds)));
  }
  return request;
}

void ExpectPoseNear(const Pose& a, const Pose& b, double tolerance = 1e-4) {
  EXPECT_NEAR((a.position - b.position).Norm(), 0.0, tolerance);
  EXPECT_NEAR(a.rotation.AngleTo(b.rotation), 0.0, tolerance);
}

const Pose kInFront{Vec3{0.8, 0.1, 0.35}, Quat::FromRpy(0.0, 0.3, 0.2)};
}  // namespace

// --- Inverse kinematics ---------------------------------------------------

TEST(ArmIk, RoundTripsAcrossTheWorkspace) {
  std::mt19937 rng(42);
  const auto lower = ArmKinematics::Lower().ToArray(), upper = ArmKinematics::Upper().ToArray();
  int solved = 0;
  for (int i = 0; i < 2000; ++i) {
    std::array<double, 6> q{};
    for (std::size_t j = 0; j < q.size(); ++j) {
      q[j] = std::uniform_real_distribution<double>(lower[j], upper[j])(rng);
    }
    const ArmJoints joints = ArmJoints::FromArray(q);
    if (std::abs(std::sin(joints.wr0)) < 1e-3) continue;  // wrist singularity: el1 / wr1 not unique
    const Pose hand = ArmKinematics::BodyTHand(joints);
    const auto solution = ArmKinematics::InverseHand(hand, joints);
    ASSERT_TRUE(solution.has_value()) << i;
    // Near the original configuration, IK returns it.
    EXPECT_LT(solution->MaxDifference(joints), 1e-6) << i;
    ExpectPoseNear(ArmKinematics::BodyTHand(*solution), hand);
    ++solved;
  }
  EXPECT_GT(solved, 1900);
}

TEST(ArmIk, PicksTheSolutionNearestThePreferredConfiguration) {
  // The same hand pose with the wrist flipped: el1 + pi, -wr0, wr1 + pi.
  const ArmJoints a{0.2, -0.8, 1.6, 1.0, -0.5, 0.4};
  const ArmJoints b{0.2, -0.8, 1.6, 1.0 - M_PI, 0.5, 0.4 - M_PI};  // both within the limits
  ExpectPoseNear(ArmKinematics::BodyTHand(a), ArmKinematics::BodyTHand(b), 1e-9);
  const Pose hand = ArmKinematics::BodyTHand(a);
  EXPECT_LT(ArmKinematics::InverseHand(hand, a)->MaxDifference(a), 1e-6);
  EXPECT_LT(ArmKinematics::InverseHand(hand, b)->MaxDifference(b), 1e-6);
}

TEST(ArmIk, OutOfReach) {
  EXPECT_FALSE(ArmKinematics::InverseHand(Pose{Vec3{2.0, 0.0, 0.3}, Quat{}}, ArmKinematics::Ready()).has_value());
  // Within reach, but the hand would have to point backward through the wrist limits.
  EXPECT_TRUE(ArmKinematics::InverseHand(kInFront, ArmKinematics::Ready()).has_value());
}

TEST(ArmIk, WristSingularityKeepsTheElbowRoll) {
  const ArmJoints straight{0.0, -0.5, 1.0, 0.7, 0.0, -0.2};
  const auto solution = ArmKinematics::InverseHand(ArmKinematics::BodyTHand(straight), straight);
  ASSERT_TRUE(solution.has_value());
  ExpectPoseNear(ArmKinematics::BodyTHand(*solution), ArmKinematics::BodyTHand(straight));
  EXPECT_NEAR(solution->el1, 0.7, 1e-9);
}

// --- Joint moves ------------------------------------------------------------

TEST(ArmMotion, JointMoveFollowsTheTrajectoryTiming) {
  ArmContext context{ArmKinematics::Stow(), Pose{}};
  api::ArmCommand::Request request;
  auto* point = request.mutable_arm_joint_move_command()->mutable_trajectory()->add_points();
  const ArmJoints ready = ArmKinematics::Ready();
  point->mutable_position()->mutable_sh0()->set_value(ready.sh0);
  point->mutable_position()->mutable_sh1()->set_value(ready.sh1);
  point->mutable_position()->mutable_el0()->set_value(ready.el0);
  point->mutable_position()->mutable_el1()->set_value(ready.el1);
  point->mutable_position()->mutable_wr0()->set_value(ready.wr0);
  point->mutable_position()->mutable_wr1()->set_value(ready.wr1);
  *point->mutable_time_since_reference() = spot_sim::ToDuration(4s);

  ArmMotion motion = Create(request, context);
  Simulate(motion, context, 0.0, 2.0);
  // Halfway in time, halfway in joint space.
  EXPECT_NEAR(context.joints.sh1, (ArmKinematics::Stow().sh1 + ready.sh1) / 2, 0.01);
  EXPECT_EQ(motion.progress(), ArmMotion::Progress::kInProgress);
  Simulate(motion, context, 2.0, 4.1);
  EXPECT_LT(context.joints.MaxDifference(ready), 1e-9);
  EXPECT_EQ(motion.progress(), ArmMotion::Progress::kComplete);

  api::ArmCommand::Feedback feedback;
  motion.FillFeedback(&feedback);
  EXPECT_EQ(feedback.arm_joint_move_feedback().status(), api::ArmJointMoveCommand::Feedback::STATUS_COMPLETE);
  EXPECT_EQ(feedback.arm_joint_move_feedback().planner_status(),
            api::ArmJointMoveCommand::Feedback::PLANNER_STATUS_SUCCESS);
}

TEST(ArmMotion, JointMoveTooFastIsStretchedToTheSpeedLimit) {
  ArmContext context{ArmKinematics::Stow(), Pose{}};
  api::ArmCommand::Request request;
  auto* point = request.mutable_arm_joint_move_command()->mutable_trajectory()->add_points();
  point->mutable_position()->mutable_sh0()->set_value(1.0);  // the other joints stay
  *point->mutable_time_since_reference() = spot_sim::ToDuration(100ms);
  ArmMotion motion = Create(request, context);
  Simulate(motion, context, 0.0, 0.5);
  EXPECT_NEAR(context.joints.sh0, 0.5, 0.02);  // 1 rad/s default
  Simulate(motion, context, 0.5, 1.1);
  EXPECT_EQ(motion.progress(), ArmMotion::Progress::kComplete);
  EXPECT_DOUBLE_EQ(context.joints.sh1, ArmKinematics::Stow().sh1);
  api::ArmCommand::Feedback feedback;
  motion.FillFeedback(&feedback);
  EXPECT_EQ(feedback.arm_joint_move_feedback().planner_status(),
            api::ArmJointMoveCommand::Feedback::PLANNER_STATUS_MODIFIED);
}

// --- Cartesian moves --------------------------------------------------------

TEST(ArmMotion, CartesianMoveInBodyReachesThePose) {
  ArmContext context{ArmKinematics::Ready(), Pose{}};
  ArmMotion motion = Create(Cartesian("body", kInFront, 2.0), context);
  Simulate(motion, context, 0.0, 1.0);
  EXPECT_EQ(motion.progress(), ArmMotion::Progress::kInProgress);
  Simulate(motion, context, 1.0, 2.1);
  EXPECT_EQ(motion.progress(), ArmMotion::Progress::kComplete);
  ExpectPoseNear(ArmKinematics::BodyTHand(context.joints), kInFront);
  api::ArmCommand::Feedback feedback;
  motion.FillFeedback(&feedback);
  EXPECT_EQ(feedback.arm_cartesian_feedback().status(), api::ArmCartesianCommand::Feedback::STATUS_TRAJECTORY_COMPLETE);
  EXPECT_LT(feedback.arm_cartesian_feedback().measured_pos_distance_to_goal(), 1e-4);
}

TEST(ArmMotion, CartesianMoveInOdomHoldsTheHandInTheWorld) {
  const Pose odom_T_body{Vec3{1.0, 2.0, 0.5}, Quat::FromYaw(0.5)};
  ArmContext context{ArmKinematics::Ready(), odom_T_body};
  const Pose odom_T_hand = odom_T_body * kInFront;
  ArmMotion motion = Create(Cartesian("odom", odom_T_hand), context);
  Simulate(motion, context, 0.0, 3.0);
  ASSERT_EQ(motion.progress(), ArmMotion::Progress::kComplete);

  // The body walks 10 cm forward and turns; the hand stays put in odom.
  context.odom_T_body = odom_T_body * Pose{Vec3{0.1, 0.0, 0.0}, Quat::FromYaw(0.1)};
  Simulate(motion, context, 3.0, 3.1);
  ExpectPoseNear(context.odom_T_body * ArmKinematics::BodyTHand(context.joints), odom_T_hand);
}

TEST(ArmMotion, UnreachablePoseStalls) {
  ArmContext context{ArmKinematics::Ready(), Pose{}};
  ArmMotion motion = Create(Cartesian("body", Pose{Vec3{2.0, 0.0, 0.3}, Quat{}}), context);
  Simulate(motion, context, 0.0, 5.0);
  EXPECT_EQ(motion.progress(), ArmMotion::Progress::kStalled);
  api::ArmCommand::Feedback feedback;
  motion.FillFeedback(&feedback);
  EXPECT_EQ(feedback.arm_cartesian_feedback().status(), api::ArmCartesianCommand::Feedback::STATUS_TRAJECTORY_STALLED);
}

TEST(ArmMotion, CartesianRejectsUnknownFramesAndForceControl) {
  const ArmContext context{ArmKinematics::Ready(), Pose{}};
  EXPECT_EQ(Rejection(Cartesian("spot/body", kInFront), context), api::RobotCommandResponse::STATUS_UNKNOWN_FRAME);
  auto force = Cartesian("body", kInFront);
  force.mutable_arm_cartesian_command()->set_z_axis(api::ArmCartesianCommand::Request::AXIS_MODE_FORCE);
  EXPECT_EQ(Rejection(force, context), api::RobotCommandResponse::STATUS_UNSUPPORTED);
}

// --- Velocity ---------------------------------------------------------------

TEST(ArmMotion, CylindricalVelocityMovesTheHandOutwardUntilEndTime) {
  ArmContext context{ArmKinematics::Ready(), Pose{}};
  api::ArmCommand::Request request;
  auto* velocity = request.mutable_arm_velocity_command();
  velocity->mutable_cylindrical_velocity()->mutable_linear_velocity()->set_r(-0.1);
  velocity->mutable_cylindrical_velocity()->mutable_linear_velocity()->set_z(0.05);
  *velocity->mutable_end_time() = spot_sim::ToTimestamp(1s);
  const Pose start = ArmKinematics::BodyTHand(context.joints);

  ArmMotion motion = Create(request, context);
  Simulate(motion, context, 0.0, 2.0);
  const Pose end = ArmKinematics::BodyTHand(context.joints);
  EXPECT_NEAR(end.position.x - start.position.x, -0.1, 1e-3);  // straight ahead: r is x
  EXPECT_NEAR(end.position.z - start.position.z, 0.05, 1e-3);
  EXPECT_NEAR(end.rotation.AngleTo(start.rotation), 0.0, 1e-6);
  EXPECT_EQ(motion.progress(), ArmMotion::Progress::kTimedOut);
}

TEST(ArmMotion, CartesianVelocityInOdom) {
  const Pose odom_T_body{Vec3{}, Quat::FromYaw(M_PI / 2)};  // body x = odom y
  ArmContext context{ArmKinematics::Ready(), odom_T_body};
  api::ArmCommand::Request request;
  auto* velocity = request.mutable_arm_velocity_command();
  velocity->mutable_cartesian_velocity()->set_frame_name("odom");
  velocity->mutable_cartesian_velocity()->mutable_velocity_in_frame_name()->set_y(-0.1);
  *velocity->mutable_end_time() = spot_sim::ToTimestamp(1s);
  const Vec3 start = ArmKinematics::BodyTHand(context.joints).position;
  ArmMotion motion = Create(request, context);
  Simulate(motion, context, 0.0, 1.0);
  EXPECT_NEAR(ArmKinematics::BodyTHand(context.joints).position.x - start.x, -0.1, 1e-3);
}

TEST(ArmMotion, VelocityNeedsEndTimeAndTimeSync) {
  const ArmContext context{ArmKinematics::Ready(), Pose{}};
  api::ArmCommand::Request request;
  request.mutable_arm_velocity_command()->mutable_cylindrical_velocity();
  EXPECT_EQ(Rejection(request, context), api::RobotCommandResponse::STATUS_INVALID_REQUEST);
  *request.mutable_arm_velocity_command()->mutable_end_time() = spot_sim::ToTimestamp(1s);
  EXPECT_EQ(Rejection(request, context, 0s, /*clock_synced=*/false), api::RobotCommandResponse::STATUS_NO_TIMESYNC);
  EXPECT_EQ(Rejection(request, context, 2s), api::RobotCommandResponse::STATUS_EXPIRED);
}

// --- Gaze ---------------------------------------------------------------------

TEST(ArmMotion, GazePointsTheHandAtTheTarget) {
  ArmContext context{ArmKinematics::Ready(), Pose{}};
  api::ArmCommand::Request request;
  auto* gaze = request.mutable_arm_gaze_command();
  gaze->set_frame1_name("body");
  const Vec3 target{1.5, 0.8, -0.3};
  *gaze->mutable_target_trajectory_in_frame1()->add_points()->mutable_point() = spot_sim::ToProto(target);
  const Vec3 hand_before = ArmKinematics::BodyTHand(context.joints).position;

  ArmMotion motion = Create(request, context);
  Simulate(motion, context, 0.0, 0.2);
  ASSERT_EQ(motion.progress(), ArmMotion::Progress::kComplete);
  const Pose hand = ArmKinematics::BodyTHand(context.joints);
  EXPECT_NEAR((hand.position - hand_before).Norm(), 0.0, 1e-4);  // the hand stays where it was
  EXPECT_NEAR(hand.rotation.XAxis().Dot((target - hand.position).Normalized()), 1.0, 1e-6);
  EXPECT_NEAR(hand.rotation.YAxis().z, 0.0, 1e-6);  // no roll
  api::ArmCommand::Feedback feedback;
  motion.FillFeedback(&feedback);
  EXPECT_TRUE(feedback.arm_gaze_feedback().gazing_at_target());
}

// --- Through the command system -------------------------------------------------

TEST(ArmMotion, CommandSystemRunsCartesianMovesAlongsideWalking) {
  spot_sim::PowerSystem power{0s, 0s};
  api::PowerCommandRequest on;
  on.set_request(api::PowerCommandRequest::REQUEST_ON_MOTORS);
  power.Command(on, api::ESTOP_LEVEL_NONE, 0s);
  power.Step(api::ESTOP_LEVEL_NONE, 0s);
  spot_sim::CommandSystem commands;
  commands.EnableArm();
  spot_sim::ArmSample arm;
  arm.joints = ArmKinematics::Ready();
  spot_sim::LocomotionStatus status;
  status.state = spot_sim::LocomotionState::kStanding;
  commands.Step(0s, api::ESTOP_LEVEL_NONE, power, status, arm);

  api::LeaseUseResult lease;
  lease.set_status(api::LeaseUseResult::STATUS_OK);
  api::RobotCommandRequest walk;
  auto* velocity = walk.mutable_command()->mutable_synchronized_command()->mutable_mobility_command()->mutable_se2_velocity_request();
  velocity->set_se2_frame_name("body");
  velocity->mutable_velocity()->mutable_linear()->set_x(0.5);
  *velocity->mutable_end_time() = spot_sim::ToTimestamp(10s);
  ASSERT_EQ(commands.Command(walk, lease, true, true, 0s).status(), api::RobotCommandResponse::STATUS_OK);

  api::RobotCommandRequest reach;
  *reach.mutable_command()->mutable_synchronized_command()->mutable_arm_command() = Cartesian("body", kInFront, 1.0);
  const auto response = commands.Command(reach, lease, true, true, 0s);
  ASSERT_EQ(response.status(), api::RobotCommandResponse::STATUS_OK) << response.message();
  EXPECT_EQ(commands.Locomotion(true).mode, spot_sim::LocomotionMode::kVelocity);

  for (int i = 1; i <= 120; ++i) {
    arm.joints = commands.Arm(true).joints;
    commands.Step(std::chrono::milliseconds(10 * i), api::ESTOP_LEVEL_NONE, power, status, arm);
  }
  api::RobotCommandFeedbackRequest request;
  request.set_robot_command_id(response.robot_command_id());
  const auto feedback = commands.Feedback(request, status).feedback().synchronized_feedback().arm_command_feedback();
  EXPECT_EQ(feedback.status(), api::RobotCommandFeedbackStatus::STATUS_PROCESSING);
  EXPECT_EQ(feedback.arm_cartesian_feedback().status(), api::ArmCartesianCommand::Feedback::STATUS_TRAJECTORY_COMPLETE);
  ExpectPoseNear(ArmKinematics::BodyTHand(arm.joints), kInFront);
}
