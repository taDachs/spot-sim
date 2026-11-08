#include <gtest/gtest.h>

#include "spot_sim_core/power.hpp"

using namespace std::chrono_literals;
namespace api = bosdyn::api;

namespace {
api::PowerCommandRequest Request(api::PowerCommandRequest::Request what) {
  api::PowerCommandRequest request;
  request.set_request(what);
  return request;
}

api::PowerCommandStatus FeedbackFor(const spot_sim::PowerSystem& power, uint32_t id) {
  api::PowerCommandFeedbackRequest request;
  request.set_power_command_id(id);
  return power.Feedback(request).status();
}
}  // namespace

TEST(Power, PowersOnAfterTransitionInRobotTime) {
  spot_sim::PowerSystem power(1s, 500ms);
  const auto response = power.Command(Request(api::PowerCommandRequest::REQUEST_ON_MOTORS), api::ESTOP_LEVEL_NONE, 10s);
  ASSERT_EQ(response.status(), api::STATUS_IN_PROGRESS);
  EXPECT_EQ(power.MotorState(), api::PowerState::MOTOR_POWER_STATE_POWERING_ON);

  power.Step(api::ESTOP_LEVEL_NONE, 10s + 500ms);
  EXPECT_EQ(FeedbackFor(power, response.power_command_id()), api::STATUS_IN_PROGRESS);
  power.Step(api::ESTOP_LEVEL_NONE, 11s);
  EXPECT_EQ(FeedbackFor(power, response.power_command_id()), api::STATUS_SUCCESS);
  EXPECT_EQ(power.MotorState(), api::PowerState::MOTOR_POWER_STATE_ON);
  EXPECT_TRUE(power.MotorsOn());
}

TEST(Power, RefusesWhileEstopped) {
  spot_sim::PowerSystem power;
  const auto response = power.Command(Request(api::PowerCommandRequest::REQUEST_ON_MOTORS), api::ESTOP_LEVEL_CUT, 0s);
  EXPECT_EQ(response.status(), api::STATUS_ESTOPPED);
  EXPECT_EQ(power.MotorState(), api::PowerState::MOTOR_POWER_STATE_OFF);
}

TEST(Power, EstopCutsMotorsAndFailsPendingPowerOn) {
  spot_sim::PowerSystem power(1s);
  const auto id =
      power.Command(Request(api::PowerCommandRequest::REQUEST_ON_MOTORS), api::ESTOP_LEVEL_NONE, 0s).power_command_id();
  power.Step(api::ESTOP_LEVEL_SETTLE_THEN_CUT, 100ms);
  EXPECT_EQ(power.MotorState(), api::PowerState::MOTOR_POWER_STATE_OFF);
  EXPECT_EQ(FeedbackFor(power, id), api::STATUS_ESTOPPED);

  power.Command(Request(api::PowerCommandRequest::REQUEST_ON_MOTORS), api::ESTOP_LEVEL_NONE, 1s);
  power.Step(api::ESTOP_LEVEL_NONE, 2s);
  ASSERT_TRUE(power.MotorsOn());
  // Settle-then-cut leaves a powered robot to sit down first (CommandSystem).
  power.Step(api::ESTOP_LEVEL_SETTLE_THEN_CUT, 2500ms);
  EXPECT_TRUE(power.MotorsOn());
  power.Step(api::ESTOP_LEVEL_CUT, 3s);
  EXPECT_FALSE(power.MotorsOn());
}

TEST(Power, PowersOffAndRepeatedRequestsSucceedImmediately) {
  spot_sim::PowerSystem power(1s, 500ms);
  power.Command(Request(api::PowerCommandRequest::REQUEST_ON_MOTORS), api::ESTOP_LEVEL_NONE, 0s);
  power.Step(api::ESTOP_LEVEL_NONE, 1s);
  EXPECT_EQ(power.Command(Request(api::PowerCommandRequest::REQUEST_ON_MOTORS), api::ESTOP_LEVEL_NONE, 1s).status(),
            api::STATUS_SUCCESS);

  const auto off = power.Command(Request(api::PowerCommandRequest::REQUEST_OFF_MOTORS), api::ESTOP_LEVEL_NONE, 2s);
  EXPECT_EQ(off.status(), api::STATUS_IN_PROGRESS);
  power.Step(api::ESTOP_LEVEL_NONE, 2s + 600ms);
  EXPECT_EQ(FeedbackFor(power, off.power_command_id()), api::STATUS_SUCCESS);
  EXPECT_EQ(power.MotorState(), api::PowerState::MOTOR_POWER_STATE_OFF);
}

TEST(Power, UnknownCommandIdReportsUnknown) {
  spot_sim::PowerSystem power;
  EXPECT_EQ(FeedbackFor(power, 999), api::STATUS_UNKNOWN);
}
