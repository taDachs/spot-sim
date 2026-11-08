#include <gtest/gtest.h>

#include "spot_sim_core/commands.hpp"

using namespace std::chrono_literals;
namespace api = bosdyn::api;
using spot_sim::LocomotionMode;
using spot_sim::LocomotionState;
using spot_sim::LocomotionStatus;

namespace {
api::LeaseUseResult LeaseOk() {
  api::LeaseUseResult result;
  result.set_status(api::LeaseUseResult::STATUS_OK);
  return result;
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

api::RobotCommandRequest SafePowerOff() {
  api::RobotCommandRequest request;
  request.mutable_command()->mutable_full_body_command()->mutable_safe_power_off_request();
  return request;
}

api::RobotCommandFeedbackRequest FeedbackFor(uint32_t id) {
  api::RobotCommandFeedbackRequest request;
  request.set_robot_command_id(id);
  return request;
}

/// Powered robot fixture.
struct Powered {
  spot_sim::PowerSystem power{0s, 0s};
  spot_sim::CommandSystem commands;
  LocomotionStatus status;

  Powered() {
    api::PowerCommandRequest on;
    on.set_request(api::PowerCommandRequest::REQUEST_ON_MOTORS);
    power.Command(on, api::ESTOP_LEVEL_NONE, 0s);
    power.Step(api::ESTOP_LEVEL_NONE, 0s);
  }

  /// Simulates an ideal base that reaches every target immediately.
  void BaseReaches(LocomotionState state) {
    status.state = state;
    status.reached_sequence = commands.Locomotion(power.MotorsOn()).sequence;
  }
};
}  // namespace

TEST(Commands, StandRequiresPowerAndLease) {
  spot_sim::CommandSystem commands;
  EXPECT_EQ(commands.Command(Stand(), LeaseOk(), /*motors_on=*/false, true, 0s).status(),
            api::RobotCommandResponse::STATUS_NOT_POWERED_ON);

  api::LeaseUseResult older;
  older.set_status(api::LeaseUseResult::STATUS_OLDER);
  const auto response = commands.Command(Stand(), older, true, true, 0s);
  EXPECT_EQ(response.lease_use_result().status(), api::LeaseUseResult::STATUS_OLDER);
  EXPECT_EQ(response.robot_command_id(), 0u);
}

TEST(Commands, StandFeedbackFollowsBase) {
  Powered robot;
  const auto response = robot.commands.Command(Stand(), LeaseOk(), true, true, 0s);
  ASSERT_EQ(response.status(), api::RobotCommandResponse::STATUS_OK);
  const auto locomotion = robot.commands.Locomotion(true);
  EXPECT_EQ(locomotion.mode, LocomotionMode::kStand);
  EXPECT_TRUE(locomotion.motors_on);

  const auto stand_status = [&] {
    return robot.commands.Feedback(FeedbackFor(response.robot_command_id()), robot.status)
        .feedback()
        .synchronized_feedback()
        .mobility_command_feedback()
        .stand_feedback()
        .status();
  };
  EXPECT_EQ(stand_status(), api::StandCommand::Feedback::STATUS_IN_PROGRESS);
  robot.BaseReaches(LocomotionState::kStanding);
  EXPECT_EQ(stand_status(), api::StandCommand::Feedback::STATUS_IS_STANDING);
}

TEST(Commands, NewCommandOverridesPrevious) {
  Powered robot;
  const auto stand = robot.commands.Command(Stand(), LeaseOk(), true, true, 0s);
  robot.commands.Command(Sit(), LeaseOk(), true, true, 0s);
  const auto feedback = robot.commands.Feedback(FeedbackFor(stand.robot_command_id()), robot.status);
  EXPECT_EQ(feedback.feedback().synchronized_feedback().mobility_command_feedback().status(),
            api::RobotCommandFeedbackStatus::STATUS_COMMAND_OVERRIDDEN);
  EXPECT_EQ(robot.commands.Locomotion(true).mode, LocomotionMode::kSit);
}

TEST(Commands, SafePowerOffSitsThenCutsPower) {
  Powered robot;
  robot.commands.Command(Stand(), LeaseOk(), true, true, 0s);
  robot.BaseReaches(LocomotionState::kStanding);

  const auto off = robot.commands.Command(SafePowerOff(), LeaseOk(), true, true, 1s);
  ASSERT_EQ(off.status(), api::RobotCommandResponse::STATUS_OK);
  EXPECT_EQ(robot.commands.Locomotion(true).mode, LocomotionMode::kSit);

  robot.commands.Step(1s, api::ESTOP_LEVEL_NONE, robot.power, robot.status);
  EXPECT_TRUE(robot.power.MotorsOn());  // still standing

  robot.BaseReaches(LocomotionState::kSitting);
  robot.commands.Step(2s, api::ESTOP_LEVEL_NONE, robot.power, robot.status);
  EXPECT_FALSE(robot.power.MotorsOn());
  EXPECT_EQ(robot.commands.Feedback(FeedbackFor(off.robot_command_id()), robot.status)
                .feedback()
                .full_body_feedback()
                .safe_power_off_feedback()
                .status(),
            api::SafePowerOffCommand::Feedback::STATUS_POWERED_OFF);
}

TEST(Commands, SafePowerOffWhenAlreadyOffIsDone) {
  spot_sim::CommandSystem commands;
  const auto off = commands.Command(SafePowerOff(), LeaseOk(), false, true, 0s);
  ASSERT_EQ(off.status(), api::RobotCommandResponse::STATUS_OK);
  EXPECT_EQ(commands.Feedback(FeedbackFor(off.robot_command_id()), {})
                .feedback()
                .full_body_feedback()
                .safe_power_off_feedback()
                .status(),
            api::SafePowerOffCommand::Feedback::STATUS_POWERED_OFF);
}

TEST(Commands, SettleThenCutSitsFirst) {
  Powered robot;
  robot.commands.Command(Stand(), LeaseOk(), true, true, 0s);
  robot.BaseReaches(LocomotionState::kStanding);

  robot.power.Step(api::ESTOP_LEVEL_SETTLE_THEN_CUT, 1s);
  robot.commands.Step(1s, api::ESTOP_LEVEL_SETTLE_THEN_CUT, robot.power, robot.status);
  EXPECT_TRUE(robot.power.MotorsOn());
  EXPECT_EQ(robot.commands.Locomotion(true).mode, LocomotionMode::kSit);

  robot.BaseReaches(LocomotionState::kSitting);
  robot.commands.Step(2s, api::ESTOP_LEVEL_SETTLE_THEN_CUT, robot.power, robot.status);
  EXPECT_FALSE(robot.power.MotorsOn());
}

TEST(Commands, SettleThenCutTimesOut) {
  Powered robot;
  robot.commands.Command(Stand(), LeaseOk(), true, true, 0s);
  robot.BaseReaches(LocomotionState::kStanding);
  robot.commands.Step(1s, api::ESTOP_LEVEL_SETTLE_THEN_CUT, robot.power, robot.status);
  robot.commands.Step(5s, api::ESTOP_LEVEL_SETTLE_THEN_CUT, robot.power, robot.status);
  EXPECT_TRUE(robot.power.MotorsOn());
  robot.commands.Step(6s, api::ESTOP_LEVEL_SETTLE_THEN_CUT, robot.power, robot.status);
  EXPECT_FALSE(robot.power.MotorsOn());
}

TEST(Commands, PowerLossOverridesActiveCommand) {
  Powered robot;
  const auto stand = robot.commands.Command(Stand(), LeaseOk(), true, true, 0s);
  robot.power.Cut();
  robot.commands.Step(1s, api::ESTOP_LEVEL_NONE, robot.power, robot.status);
  EXPECT_EQ(robot.commands.Feedback(FeedbackFor(stand.robot_command_id()), robot.status)
                .feedback()
                .synchronized_feedback()
                .mobility_command_feedback()
                .status(),
            api::RobotCommandFeedbackStatus::STATUS_COMMAND_OVERRIDDEN);
  EXPECT_FALSE(robot.commands.Locomotion(false).motors_on);
  EXPECT_EQ(robot.commands.Locomotion(false).mode, LocomotionMode::kSit);
}

TEST(Commands, RejectsArmAndUnsimulatedCommands) {
  spot_sim::CommandSystem commands;
  api::RobotCommandRequest arm;
  arm.mutable_command()->mutable_synchronized_command()->mutable_arm_command();
  EXPECT_EQ(commands.Command(arm, LeaseOk(), true, true, 0s).status(), api::RobotCommandResponse::STATUS_UNSUPPORTED);
  api::RobotCommandRequest selfright;
  selfright.mutable_command()->mutable_full_body_command()->mutable_selfright_request();
  EXPECT_EQ(commands.Command(selfright, LeaseOk(), true, true, 0s).status(),
            api::RobotCommandResponse::STATUS_UNSUPPORTED);
  EXPECT_EQ(commands.Command(api::RobotCommandRequest{}, LeaseOk(), true, true, 0s).status(),
            api::RobotCommandResponse::STATUS_INVALID_REQUEST);
}
