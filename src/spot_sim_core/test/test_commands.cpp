#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include <bosdyn/api/spot/robot_command.pb.h>

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

/// se2_velocity_request as the SDK builds it (synchro_velocity_command).
api::RobotCommandRequest Velocity(double v_x, double v_y, double v_rot, spot_sim::RobotTime end_time,
                                  const std::string& frame = "body") {
  api::RobotCommandRequest request;
  auto* mobility = request.mutable_command()->mutable_synchronized_command()->mutable_mobility_command();
  auto* velocity = mobility->mutable_se2_velocity_request();
  *velocity->mutable_end_time() = spot_sim::ToTimestamp(end_time);
  velocity->set_se2_frame_name(frame);
  velocity->mutable_velocity()->mutable_linear()->set_x(v_x);
  velocity->mutable_velocity()->mutable_linear()->set_y(v_y);
  velocity->mutable_velocity()->set_angular(v_rot);
  velocity->mutable_slew_rate_limit()->mutable_linear()->set_x(4.0);
  velocity->mutable_slew_rate_limit()->mutable_linear()->set_y(4.0);
  velocity->mutable_slew_rate_limit()->set_angular(2.0);
  mobility->mutable_params()->PackFrom(api::spot::MobilityParams{});
  return request;
}

api::RobotCommandFeedbackStatus::Status MobilityStatus(const api::RobotCommandFeedbackResponse& response) {
  return response.feedback().synchronized_feedback().mobility_command_feedback().status();
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

TEST(Commands, VelocityDrivesBaseUntilEndTime) {
  Powered robot;
  const auto response = robot.commands.Command(Velocity(0.5, 0.1, -0.3, 2s), LeaseOk(), true, true, 1s);
  ASSERT_EQ(response.status(), api::RobotCommandResponse::STATUS_OK) << response.message();
  auto locomotion = robot.commands.Locomotion(true);
  EXPECT_EQ(locomotion.mode, LocomotionMode::kVelocity);
  EXPECT_DOUBLE_EQ(locomotion.v_x, 0.5);
  EXPECT_DOUBLE_EQ(locomotion.v_y, 0.1);
  EXPECT_DOUBLE_EQ(locomotion.v_rot, -0.3);
  // Slew rate limits from the request cap the base's acceleration.
  EXPECT_LE(locomotion.max_accel_rot, 2.0);

  const auto feedback = FeedbackFor(response.robot_command_id());
  robot.commands.Step(1500ms, api::ESTOP_LEVEL_NONE, robot.power, robot.status);
  EXPECT_EQ(MobilityStatus(robot.commands.Feedback(feedback, robot.status)),
            api::RobotCommandFeedbackStatus::STATUS_PROCESSING);
  EXPECT_TRUE(robot.commands.Feedback(feedback, robot.status)
                  .feedback()
                  .synchronized_feedback()
                  .mobility_command_feedback()
                  .has_se2_velocity_feedback());

  robot.commands.Step(2s, api::ESTOP_LEVEL_NONE, robot.power, robot.status);
  locomotion = robot.commands.Locomotion(true);
  EXPECT_EQ(locomotion.mode, LocomotionMode::kStand);
  EXPECT_EQ(locomotion.v_x, 0.0);
  EXPECT_EQ(locomotion.v_rot, 0.0);
  EXPECT_EQ(MobilityStatus(robot.commands.Feedback(feedback, robot.status)),
            api::RobotCommandFeedbackStatus::STATUS_COMMAND_TIMED_OUT);
}

TEST(Commands, NewVelocityCommandOverridesPrevious) {
  Powered robot;
  const auto first = robot.commands.Command(Velocity(0.5, 0, 0, 2s), LeaseOk(), true, true, 1s);
  const auto second = robot.commands.Command(Velocity(0.2, 0, 0, 3s), LeaseOk(), true, true, 1100ms);
  robot.commands.Step(2500ms, api::ESTOP_LEVEL_NONE, robot.power, robot.status);
  EXPECT_EQ(MobilityStatus(robot.commands.Feedback(FeedbackFor(first.robot_command_id()), robot.status)),
            api::RobotCommandFeedbackStatus::STATUS_COMMAND_OVERRIDDEN);
  EXPECT_EQ(MobilityStatus(robot.commands.Feedback(FeedbackFor(second.robot_command_id()), robot.status)),
            api::RobotCommandFeedbackStatus::STATUS_PROCESSING);
  EXPECT_DOUBLE_EQ(robot.commands.Locomotion(true).v_x, 0.2);
}

TEST(Commands, VelocityValidation) {
  Powered robot;
  EXPECT_EQ(robot.commands.Command(Velocity(0.5, 0, 0, 2s), LeaseOk(), true, /*clock_synced=*/false, 1s).status(),
            api::RobotCommandResponse::STATUS_NO_TIMESYNC);
  EXPECT_EQ(robot.commands.Command(Velocity(0.5, 0, 0, 1s), LeaseOk(), true, true, 1s).status(),
            api::RobotCommandResponse::STATUS_EXPIRED);
  EXPECT_EQ(robot.commands.Command(Velocity(0.5, 0, 0, 2s, "gpe"), LeaseOk(), true, true, 1s).status(),
            api::RobotCommandResponse::STATUS_UNKNOWN_FRAME);
  EXPECT_EQ(robot.commands.Command(Velocity(0.5, 0, 0, 2s), LeaseOk(), /*motors_on=*/false, true, 1s).status(),
            api::RobotCommandResponse::STATUS_NOT_POWERED_ON);
  auto no_end_time = Velocity(0.5, 0, 0, 2s);
  no_end_time.mutable_command()
      ->mutable_synchronized_command()
      ->mutable_mobility_command()
      ->mutable_se2_velocity_request()
      ->clear_end_time();
  EXPECT_EQ(robot.commands.Command(no_end_time, LeaseOk(), true, true, 1s).status(),
            api::RobotCommandResponse::STATUS_INVALID_REQUEST);
  // None of the rejected commands changed what the base does.
  EXPECT_EQ(robot.commands.Locomotion(true).mode, LocomotionMode::kSit);
}

TEST(Commands, VelocityIsClampedToLimits) {
  Powered robot;
  const auto robot_limits = spot_sim::VelocityLimits::Robot();
  robot.commands.Command(Velocity(10.0, -10.0, 10.0, 2s), LeaseOk(), true, true, 1s);
  auto locomotion = robot.commands.Locomotion(true);
  EXPECT_DOUBLE_EQ(locomotion.v_x, robot_limits.max_x);
  EXPECT_DOUBLE_EQ(locomotion.v_y, robot_limits.min_y);
  EXPECT_DOUBLE_EQ(locomotion.v_rot, robot_limits.max_rot);

  // MobilityParams.vel_limit, as set by the driver's max_velocity service.
  auto limited = Velocity(1.0, 0.5, -1.0, 2s);
  api::spot::MobilityParams params;
  auto* vel_limit = params.mutable_vel_limit();
  vel_limit->mutable_max_vel()->mutable_linear()->set_x(0.3);
  vel_limit->mutable_max_vel()->mutable_linear()->set_y(0.2);
  vel_limit->mutable_max_vel()->set_angular(0.4);
  vel_limit->mutable_min_vel()->mutable_linear()->set_x(-0.3);
  vel_limit->mutable_min_vel()->mutable_linear()->set_y(-0.2);
  vel_limit->mutable_min_vel()->set_angular(-0.4);
  limited.mutable_command()->mutable_synchronized_command()->mutable_mobility_command()->mutable_params()->PackFrom(
      params);
  robot.commands.Command(limited, LeaseOk(), true, true, 1s);
  locomotion = robot.commands.Locomotion(true);
  EXPECT_DOUBLE_EQ(locomotion.v_x, 0.3);
  EXPECT_DOUBLE_EQ(locomotion.v_y, 0.2);
  EXPECT_DOUBLE_EQ(locomotion.v_rot, -0.4);
}

TEST(Commands, OdomFrameVelocityIsRotatedIntoBody) {
  Powered robot;
  // Facing +y in odom; moving along odom +y is moving forward.
  robot.commands.Command(Velocity(0.0, 0.5, 0.0, 2s, "odom"), LeaseOk(), true, true, 1s, M_PI / 2);
  const auto locomotion = robot.commands.Locomotion(true);
  EXPECT_NEAR(locomotion.v_x, 0.5, 1e-12);
  EXPECT_NEAR(locomotion.v_y, 0.0, 1e-12);
}

TEST(Commands, StopZeroesVelocity) {
  Powered robot;
  robot.commands.Command(Velocity(0.5, 0, 0.2, 5s), LeaseOk(), true, true, 1s);
  api::RobotCommandRequest stop;
  stop.mutable_command()->mutable_synchronized_command()->mutable_mobility_command()->mutable_stop_request();
  robot.commands.Command(stop, LeaseOk(), true, true, 1100ms);
  const auto locomotion = robot.commands.Locomotion(true);
  EXPECT_EQ(locomotion.mode, LocomotionMode::kStop);
  EXPECT_EQ(locomotion.v_x, 0.0);
  EXPECT_EQ(locomotion.v_rot, 0.0);
}
