// The Spot Arm: kinematics, its parts of RobotState, leases and directory,
// and arm / gripper commands.

#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "spot_sim_core/arm_kinematics.hpp"
#include "spot_sim_core/commands.hpp"
#include "spot_sim_core/directory.hpp"
#include "spot_sim_core/lease.hpp"
#include "spot_sim_core/robot.hpp"
#include "spot_sim_core/robot_state.hpp"

using namespace std::chrono_literals;
namespace api = bosdyn::api;
using spot_sim::ArmJoints;
using spot_sim::ArmKinematics;
using spot_sim::LocomotionMode;
using spot_sim::LocomotionState;

namespace {
api::LeaseUseResult LeaseOk() {
  api::LeaseUseResult result;
  result.set_status(api::LeaseUseResult::STATUS_OK);
  return result;
}

api::RobotCommandRequest Named(api::NamedArmPositionsCommand::Positions position) {
  api::RobotCommandRequest request;
  request.mutable_command()
      ->mutable_synchronized_command()
      ->mutable_arm_command()
      ->mutable_named_arm_position_command()
      ->set_position(position);
  return request;
}

api::RobotCommandRequest Gripper(double angle) {
  api::RobotCommandRequest request;
  request.mutable_command()
      ->mutable_synchronized_command()
      ->mutable_gripper_command()
      ->mutable_claw_gripper_command()
      ->mutable_trajectory()
      ->add_points()
      ->set_point(angle);
  return request;
}

api::RobotCommandRequest Stand() {
  api::RobotCommandRequest request;
  request.mutable_command()->mutable_synchronized_command()->mutable_mobility_command()->mutable_stand_request();
  return request;
}

api::RobotCommandRequest Sit() {
  api::RobotCommandRequest request;
  request.mutable_command()->mutable_synchronized_command()->mutable_mobility_command()->mutable_sit_request();
  return request;
}

api::RobotCommandFeedbackRequest FeedbackFor(uint32_t id) {
  api::RobotCommandFeedbackRequest request;
  request.set_robot_command_id(id);
  return request;
}

/// Powered robot with an arm whose plugin reaches every setpoint at once.
struct PoweredArm {
  spot_sim::PowerSystem power{0s, 0s};
  spot_sim::CommandSystem commands;
  spot_sim::LocomotionStatus status;
  spot_sim::ArmSample arm;

  PoweredArm() {
    commands.EnableArm();
    arm.joints = ArmKinematics::Stow();
    api::PowerCommandRequest on;
    on.set_request(api::PowerCommandRequest::REQUEST_ON_MOTORS);
    power.Command(on, api::ESTOP_LEVEL_NONE, 0s);
    power.Step(api::ESTOP_LEVEL_NONE, 0s);
    Step();
  }

  api::RobotCommandResponse Command(const api::RobotCommandRequest& request) {
    return commands.Command(request, LeaseOk(), power.MotorsOn(), true, 0s);
  }

  void Step() { commands.Step(0s, api::ESTOP_LEVEL_NONE, power, status, arm); }

  /// The arm plugin gets to the setpoint; the base to its target.
  void Settle() {
    const auto target = commands.Arm(power.MotorsOn());
    arm.joints = target.joints;
    arm.gripper = target.gripper;
    const auto locomotion = commands.Locomotion(power.MotorsOn());
    if (locomotion.mode == LocomotionMode::kSit) status.state = LocomotionState::kSitting;
    if (locomotion.mode == LocomotionMode::kStand) status.state = LocomotionState::kStanding;
    status.reached_sequence = locomotion.sequence;
    Step();
  }

  api::RobotCommandFeedbackResponse Feedback(uint32_t id) { return commands.Feedback(FeedbackFor(id), status); }
};

void ExpectNear(const spot_sim::Vec3& a, const spot_sim::Vec3& b, double tolerance = 1e-6) {
  EXPECT_NEAR(a.x, b.x, tolerance);
  EXPECT_NEAR(a.y, b.y, tolerance);
  EXPECT_NEAR(a.z, b.z, tolerance);
}
}  // namespace

// --- Kinematics ---------------------------------------------------------

TEST(ArmKinematics, ZeroPoseStretchesTheArmForward) {
  // All joints 0: upper arm, forearm and hand along +x from the shoulder.
  const auto hand = ArmKinematics::BodyTHand(ArmJoints{});
  ExpectNear(hand.position, {ArmKinematics::kShoulderX + ArmKinematics::kUpperArm + ArmKinematics::kForearmX +
                                 ArmKinematics::kHandOffset,
                             0.0, ArmKinematics::kShoulderZ + ArmKinematics::kForearmZ});
}

TEST(ArmKinematics, JointAxesTurnTheWayTheUrdfSays) {
  // sh0 (yaw, +z): positive turns the arm to the left.
  EXPECT_GT(ArmKinematics::BodyTHand(ArmJoints{0.5, 0, 0, 0, 0, 0}).position.y, 0.1);
  // sh1 (pitch, +y): negative raises the arm.
  EXPECT_GT(ArmKinematics::BodyTHand(ArmJoints{0, -0.5, 0, 0, 0, 0}).position.z, 0.5);
  // wr1 (roll, +x): rolls the hand without moving it.
  const auto rolled = ArmKinematics::BodyTHand(ArmJoints{0, 0, 0, 0, 0, 1.0});
  ExpectNear(rolled.position, ArmKinematics::BodyTHand(ArmJoints{}).position);
  EXPECT_NEAR(rolled.rotation.Roll(), 1.0, 1e-9);
}

TEST(ArmKinematics, NamedPositionsAreWithinLimitsAndKeepTheHandLevel) {
  for (const auto& q : {ArmKinematics::Stow(), ArmKinematics::Ready(), ArmKinematics::Carry()}) {
    EXPECT_EQ(ArmKinematics::Clamped(q), q);
    const auto hand = ArmKinematics::BodyTHand(q);
    EXPECT_NEAR(hand.rotation.Pitch(), 0.0, 1e-9);
    EXPECT_NEAR(hand.position.y, 0.0, 1e-9);
  }
}

TEST(ArmKinematics, StowRestsOnTheBodyAndReadyReachesOut) {
  const auto stow = ArmKinematics::BodyTHand(ArmKinematics::Stow()).position;
  const auto ready = ArmKinematics::BodyTHand(ArmKinematics::Ready()).position;
  const auto carry = ArmKinematics::BodyTHand(ArmKinematics::Carry()).position;
  // Stowed: above the body (top at ~0.1 m), the hand short of the nose (~0.55 m).
  EXPECT_GT(stow.z, 0.15);
  EXPECT_LT(stow.z, 0.35);
  EXPECT_LT(stow.x, 0.6);
  // Ready: in front of the body, carry in between.
  EXPECT_GT(ready.x, 0.75);
  EXPECT_GT(carry.x, stow.x);
  EXPECT_LT(carry.x, ready.x);
}

TEST(ArmKinematics, GripperOpenPercentage) {
  EXPECT_DOUBLE_EQ(ArmKinematics::GripperOpenPercentage(0.0), 0.0);
  EXPECT_DOUBLE_EQ(ArmKinematics::GripperOpenPercentage(ArmKinematics::kGripperOpen), 100.0);
  EXPECT_NEAR(ArmKinematics::GripperOpenPercentage(ArmKinematics::kGripperOpen / 2), 50.0, 1e-9);
  EXPECT_DOUBLE_EQ(ArmKinematics::GripperOpenPercentage(-1.5708), 100.0);  // the SDK's "open" angle
}

// --- RobotState, leases, directory ---------------------------------------

TEST(ArmState, NoArmNoManipulatorState) {
  const auto state = spot_sim::BuildRobotState(spot_sim::SittingSnapshot(1s));
  EXPECT_FALSE(state.has_manipulator_state());
  EXPECT_EQ(state.kinematic_state().transforms_snapshot().child_to_parent_edge_map().count("hand"), 0u);
}

TEST(ArmState, ArmJointsManipulatorStateAndHandFrame) {
  auto snapshot = spot_sim::SittingSnapshot(1s);
  snapshot.arm = spot_sim::ArmSample{};
  snapshot.arm->joints = ArmKinematics::Ready();
  snapshot.arm->gripper = ArmKinematics::kGripperOpen;
  const auto state = spot_sim::BuildRobotState(snapshot);

  ASSERT_TRUE(state.has_manipulator_state());
  EXPECT_DOUBLE_EQ(state.manipulator_state().gripper_open_percentage(), 100.0);
  EXPECT_EQ(state.manipulator_state().stow_state(), api::ManipulatorState::STOWSTATE_DEPLOYED);
  EXPECT_FALSE(state.manipulator_state().is_gripper_holding_item());

  // 12 leg joints, then arm0.sh0 .. arm0.f1x with Spot's names.
  const auto& joints = state.kinematic_state().joint_states();
  ASSERT_EQ(joints.size(), 20);
  EXPECT_EQ(joints[12].name(), "arm0.sh0");
  EXPECT_EQ(joints[14].name(), "arm0.hr0");
  EXPECT_DOUBLE_EQ(joints[13].position().value(), ArmKinematics::Ready().sh1);
  EXPECT_DOUBLE_EQ(joints[15].position().value(), ArmKinematics::Ready().el0);
  EXPECT_EQ(joints[19].name(), "arm0.f1x");
  EXPECT_DOUBLE_EQ(joints[19].position().value(), ArmKinematics::kGripperOpen);

  const auto& edges = state.kinematic_state().transforms_snapshot().child_to_parent_edge_map();
  ASSERT_EQ(edges.count("hand"), 1u);
  EXPECT_EQ(edges.at("hand").parent_frame_name(), "body");
  EXPECT_NEAR(edges.at("hand").parent_tform_child().position().x(),
              ArmKinematics::BodyTHand(ArmKinematics::Ready()).position.x, 1e-9);

  snapshot.arm->joints = ArmKinematics::Stow();
  EXPECT_EQ(spot_sim::BuildRobotState(snapshot).manipulator_state().stow_state(),
            api::ManipulatorState::STOWSTATE_STOWED);
}

TEST(ArmState, RobotWithArmReportsGraspOverrides) {
  spot_sim::RobotConfig config;
  config.has_arm = true;
  spot_sim::Robot robot(config);
  ASSERT_TRUE(robot.State().has_manipulator_state());
  EXPECT_EQ(robot.State().manipulator_state().stow_state(), api::ManipulatorState::STOWSTATE_STOWED);
  robot.WithFirmware([](spot_sim::Firmware& fw) {
    fw.grasp.holding_item = true;
    fw.grasp.carry_state = api::ManipulatorState::CARRY_STATE_CARRIABLE;
  });
  EXPECT_TRUE(robot.State().manipulator_state().is_gripper_holding_item());
  EXPECT_EQ(robot.State().manipulator_state().carry_state(), api::ManipulatorState::CARRY_STATE_CARRIABLE);
}

TEST(ArmState, LeaseTreeHasArmResources) {
  spot_sim::LeaseManager leases;
  api::AcquireLeaseRequest acquire;
  acquire.set_resource("arm");
  EXPECT_EQ(leases.Acquire(acquire, {}).status(), api::AcquireLeaseResponse::STATUS_INVALID_RESOURCE);

  leases.EnableArm();
  const auto list = leases.List({});
  ASSERT_EQ(list.resource_tree().sub_resources_size(), 1);
  const auto& body = list.resource_tree().sub_resources(0);
  ASSERT_EQ(body.sub_resources_size(), 2);
  EXPECT_EQ(body.sub_resources(1).resource(), "full-arm");
  EXPECT_EQ(body.sub_resources(1).sub_resources_size(), 2);
  EXPECT_EQ(list.resources_size(), 5);

  acquire.set_resource("body");
  const auto lease = leases.Acquire(acquire, {}).lease();
  api::Lease gripper = lease;
  gripper.set_resource("gripper");
  EXPECT_EQ(leases.Use(gripper, {}).status(), api::LeaseUseResult::STATUS_OK);
}

TEST(ArmState, DirectoryListsArmServicesOnlyWithAnArm) {
  for (const char* name : {"manipulation", "arm-surface-contact", "gripper-camera-param"}) {
    EXPECT_FALSE(spot_sim::FindServiceEntry(spot_sim::DefaultServiceEntries(false), name).has_value()) << name;
    EXPECT_TRUE(spot_sim::FindServiceEntry(spot_sim::DefaultServiceEntries(true), name).has_value()) << name;
  }
}

// --- Commands ------------------------------------------------------------

TEST(ArmCommands, RejectedWithoutArmOrPower) {
  spot_sim::CommandSystem no_arm;
  EXPECT_EQ(no_arm.Command(Named(api::NamedArmPositionsCommand::POSITIONS_READY), LeaseOk(), true, true, 0s).status(),
            api::RobotCommandResponse::STATUS_UNSUPPORTED);

  spot_sim::CommandSystem arm;
  arm.EnableArm();
  EXPECT_EQ(arm.Command(Named(api::NamedArmPositionsCommand::POSITIONS_READY), LeaseOk(), false, true, 0s).status(),
            api::RobotCommandResponse::STATUS_NOT_POWERED_ON);
  EXPECT_EQ(arm.Command(Named(api::NamedArmPositionsCommand::POSITIONS_UNKNOWN), LeaseOk(), true, true, 0s).status(),
            api::RobotCommandResponse::STATUS_INVALID_REQUEST);
}

TEST(ArmCommands, NamedPositionsMoveTheArmAndCompleteOnArrival) {
  PoweredArm robot;
  EXPECT_EQ(robot.commands.Arm(true).joints, ArmKinematics::Stow());  // boots stowed

  const auto ready = robot.Command(Named(api::NamedArmPositionsCommand::POSITIONS_READY));
  ASSERT_EQ(ready.status(), api::RobotCommandResponse::STATUS_OK);
  EXPECT_EQ(robot.commands.Arm(true).joints, ArmKinematics::Ready());
  const auto named_status = [&](uint32_t id) {
    return robot.Feedback(id).feedback().synchronized_feedback().arm_command_feedback().named_arm_position_feedback().status();
  };
  robot.Step();
  EXPECT_EQ(named_status(ready.robot_command_id()), api::NamedArmPositionsCommand::Feedback::STATUS_IN_PROGRESS);
  robot.Settle();
  EXPECT_EQ(named_status(ready.robot_command_id()), api::NamedArmPositionsCommand::Feedback::STATUS_COMPLETE);
  EXPECT_EQ(robot.Feedback(ready.robot_command_id()).feedback().synchronized_feedback().arm_command_feedback().status(),
            api::RobotCommandFeedbackStatus::STATUS_PROCESSING);

  const auto carry = robot.Command(Named(api::NamedArmPositionsCommand::POSITIONS_CARRY));
  EXPECT_EQ(robot.commands.Arm(true).joints, ArmKinematics::Carry());
  EXPECT_EQ(robot.Feedback(ready.robot_command_id()).feedback().synchronized_feedback().arm_command_feedback().status(),
            api::RobotCommandFeedbackStatus::STATUS_COMMAND_OVERRIDDEN);
  robot.Settle();
  EXPECT_EQ(named_status(carry.robot_command_id()), api::NamedArmPositionsCommand::Feedback::STATUS_COMPLETE);
}

TEST(ArmCommands, ArmOnlyCommandsLeaveMobilityAlone) {
  PoweredArm robot;
  const auto stand = robot.Command(Stand());
  robot.Settle();
  robot.Command(Named(api::NamedArmPositionsCommand::POSITIONS_READY));
  const auto feedback = robot.Feedback(stand.robot_command_id()).feedback().synchronized_feedback();
  EXPECT_EQ(feedback.mobility_command_feedback().status(), api::RobotCommandFeedbackStatus::STATUS_PROCESSING);
  EXPECT_EQ(feedback.mobility_command_feedback().stand_feedback().status(),
            api::StandCommand::Feedback::STATUS_IS_STANDING);
  EXPECT_EQ(robot.commands.Locomotion(true).mode, LocomotionMode::kStand);
}

TEST(ArmCommands, ArmStopHoldsTheMeasuredPose) {
  PoweredArm robot;
  robot.Command(Named(api::NamedArmPositionsCommand::POSITIONS_READY));
  robot.arm.joints = ArmJoints{0.0, -2.0, 2.5, 0.0, -0.5, 0.0};  // on its way
  robot.Step();
  api::RobotCommandRequest stop;
  stop.mutable_command()->mutable_synchronized_command()->mutable_arm_command()->mutable_arm_stop_command();
  const auto response = robot.Command(stop);
  ASSERT_EQ(response.status(), api::RobotCommandResponse::STATUS_OK);
  EXPECT_EQ(robot.commands.Arm(true).joints, robot.arm.joints);
  EXPECT_TRUE(robot.Feedback(response.robot_command_id())
                  .feedback()
                  .synchronized_feedback()
                  .arm_command_feedback()
                  .has_arm_stop_feedback());
}

TEST(ArmCommands, ClawGripperGoesToTheTrajectoryEnd) {
  PoweredArm robot;
  const auto open = robot.Command(Gripper(-1.5708));
  ASSERT_EQ(open.status(), api::RobotCommandResponse::STATUS_OK);
  EXPECT_DOUBLE_EQ(robot.commands.Arm(true).gripper, ArmKinematics::kGripperOpen);  // clamped to the joint limit
  const auto claw_status = [&](uint32_t id) {
    return robot.Feedback(id).feedback().synchronized_feedback().gripper_command_feedback().claw_gripper_feedback().status();
  };
  EXPECT_EQ(claw_status(open.robot_command_id()), api::ClawGripperCommand::Feedback::STATUS_IN_PROGRESS);
  robot.Settle();
  EXPECT_EQ(claw_status(open.robot_command_id()), api::ClawGripperCommand::Feedback::STATUS_AT_GOAL);

  // Gripper and arm are separate parts: a gripper command does not override the arm.
  const auto ready = robot.Command(Named(api::NamedArmPositionsCommand::POSITIONS_READY));
  robot.Command(Gripper(0.0));
  EXPECT_EQ(robot.Feedback(ready.robot_command_id()).feedback().synchronized_feedback().arm_command_feedback().status(),
            api::RobotCommandFeedbackStatus::STATUS_PROCESSING);
  EXPECT_EQ(robot.commands.Arm(true).joints, ArmKinematics::Ready());
}

TEST(ArmCommands, SittingStowsTheArmFirst) {
  PoweredArm robot;
  robot.Command(Stand());
  robot.Settle();
  const auto ready = robot.Command(Named(api::NamedArmPositionsCommand::POSITIONS_READY));
  robot.Settle();

  const auto sit = robot.Command(Sit());
  // The arm heads for stow; the base keeps standing until it gets there.
  EXPECT_EQ(robot.commands.Arm(true).joints, ArmKinematics::Stow());
  EXPECT_EQ(robot.commands.Locomotion(true).mode, LocomotionMode::kStop);
  EXPECT_EQ(robot.Feedback(ready.robot_command_id()).feedback().synchronized_feedback().arm_command_feedback().status(),
            api::RobotCommandFeedbackStatus::STATUS_COMMAND_OVERRIDDEN);

  robot.arm.joints = ArmKinematics::Stow();
  robot.Step();
  EXPECT_EQ(robot.commands.Locomotion(true).mode, LocomotionMode::kSit);
  robot.Settle();
  EXPECT_EQ(robot.Feedback(sit.robot_command_id())
                .feedback()
                .synchronized_feedback()
                .mobility_command_feedback()
                .sit_feedback()
                .status(),
            api::SitCommand::Feedback::STATUS_IS_SITTING);
}

TEST(ArmCommands, PowerLossFreezesTheArm) {
  PoweredArm robot;
  const auto ready = robot.Command(Named(api::NamedArmPositionsCommand::POSITIONS_READY));
  robot.arm.joints = ArmJoints{0.0, -2.0, 2.5, 0.0, -0.5, 0.0};
  robot.power.Cut();
  robot.Step();
  EXPECT_FALSE(robot.commands.Arm(robot.power.MotorsOn()).motors_on);
  EXPECT_EQ(robot.commands.Arm(false).joints, robot.arm.joints);
  EXPECT_EQ(robot.Feedback(ready.robot_command_id()).feedback().synchronized_feedback().arm_command_feedback().status(),
            api::RobotCommandFeedbackStatus::STATUS_COMMAND_OVERRIDDEN);
}
