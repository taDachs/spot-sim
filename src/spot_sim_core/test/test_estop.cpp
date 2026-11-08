#include <gtest/gtest.h>

#include <optional>

#include "spot_sim_core/estop.hpp"

using namespace std::chrono_literals;
namespace api = bosdyn::api;

namespace {
const spot_sim::WallTime kT0{};

/// Mirrors bosdyn.client.estop.EstopEndpoint.force_simple_setup() + register().
class EstopClient {
 public:
  explicit EstopClient(spot_sim::EstopSystem& system, double timeout_sec = 9.0) : system_(system) {
    endpoint_.set_role("PDB_rooted");
    endpoint_.set_name("spot-ros2");
    *endpoint_.mutable_timeout() = spot_sim::ToDuration(std::chrono::nanoseconds(static_cast<int64_t>(timeout_sec * 1e9)));
  }

  void Setup() {
    api::GetEstopConfigRequest get;
    const auto active = system_.GetConfig(get).active_config();
    api::SetEstopConfigRequest set;
    *set.mutable_config()->add_endpoints() = endpoint_;
    set.set_target_config_id(active.unique_id());
    const auto set_response = system_.SetConfig(set, /*motors_on=*/false);
    ASSERT_EQ(set_response.status(), api::SetEstopConfigResponse::STATUS_SUCCESS);
    endpoint_.set_unique_id(set_response.active_config().endpoints(0).unique_id());

    api::RegisterEstopEndpointRequest reg;
    reg.set_target_config_id(set_response.active_config().unique_id());
    reg.mutable_target_endpoint()->set_role(endpoint_.role());
    *reg.mutable_new_endpoint() = endpoint_;
    ASSERT_EQ(system_.Register(reg).status(), api::RegisterEstopEndpointResponse::STATUS_SUCCESS);
  }

  api::EstopCheckInResponse::Status CheckIn(api::EstopStopLevel level, spot_sim::WallTime now) {
    api::EstopCheckInRequest request;
    *request.mutable_endpoint() = endpoint_;
    request.set_stop_level(level);
    if (challenge_) {
      request.set_challenge(*challenge_);
      request.set_response(~*challenge_);
    }
    const auto response = system_.CheckIn(request, now);
    challenge_ = response.challenge();
    return response.status();
  }

 private:
  spot_sim::EstopSystem& system_;
  api::EstopEndpoint endpoint_;
  std::optional<uint64_t> challenge_;
};
}  // namespace

TEST(Estop, NoEndpointsMeansNotStopped) {
  spot_sim::EstopSystem system;
  EXPECT_EQ(system.StopLevel(kT0), api::ESTOP_LEVEL_NONE);
  EXPECT_TRUE(system.States(kT0).empty());
}

TEST(Estop, RegisteredButSilentEndpointCutsUntilCheckIn) {
  spot_sim::EstopSystem system;
  EstopClient client(system);
  client.Setup();
  EXPECT_EQ(system.StopLevel(kT0), api::ESTOP_LEVEL_CUT);
  EXPECT_EQ(client.CheckIn(api::ESTOP_LEVEL_NONE, kT0), api::EstopCheckInResponse::STATUS_OK);
  EXPECT_EQ(system.StopLevel(kT0), api::ESTOP_LEVEL_NONE);
}

TEST(Estop, ChallengeResponseIsVerified) {
  spot_sim::EstopSystem system;
  EstopClient client(system);
  client.Setup();
  ASSERT_EQ(client.CheckIn(api::ESTOP_LEVEL_NONE, kT0), api::EstopCheckInResponse::STATUS_OK);
  EXPECT_EQ(client.CheckIn(api::ESTOP_LEVEL_NONE, kT0 + 1s), api::EstopCheckInResponse::STATUS_OK);

  api::EstopCheckInRequest bogus;
  *bogus.mutable_endpoint() = system.GetConfig({}).active_config().endpoints(0);
  bogus.set_stop_level(api::ESTOP_LEVEL_NONE);
  bogus.set_response(12345);
  EXPECT_EQ(system.CheckIn(bogus, kT0 + 2s).status(), api::EstopCheckInResponse::STATUS_INCORRECT_CHALLENGE_RESPONSE);
}

TEST(Estop, TimeoutCutsAndCheckInLevelsApply) {
  spot_sim::EstopSystem system;
  EstopClient client(system, 9.0);
  client.Setup();
  client.CheckIn(api::ESTOP_LEVEL_NONE, kT0);
  EXPECT_EQ(system.StopLevel(kT0 + 8s), api::ESTOP_LEVEL_NONE);
  EXPECT_EQ(system.StopLevel(kT0 + 10s), api::ESTOP_LEVEL_CUT);
  ASSERT_EQ(system.States(kT0 + 10s).size(), 1u);
  EXPECT_TRUE(system.States(kT0 + 10s)[0].estopped);

  client.CheckIn(api::ESTOP_LEVEL_SETTLE_THEN_CUT, kT0 + 11s);
  EXPECT_EQ(system.StopLevel(kT0 + 11s), api::ESTOP_LEVEL_SETTLE_THEN_CUT);
  client.CheckIn(api::ESTOP_LEVEL_NONE, kT0 + 12s);
  EXPECT_EQ(system.StopLevel(kT0 + 12s), api::ESTOP_LEVEL_NONE);
}

TEST(Estop, ConfigChangeRefusedWithMotorsOn) {
  spot_sim::EstopSystem system;
  api::SetEstopConfigRequest set;
  set.set_target_config_id(system.GetConfig({}).active_config().unique_id());
  EXPECT_EQ(system.SetConfig(set, /*motors_on=*/true).status(), api::SetEstopConfigResponse::STATUS_MOTORS_ON);
  set.set_target_config_id("stale");
  EXPECT_EQ(system.SetConfig(set, false).status(), api::SetEstopConfigResponse::STATUS_INVALID_ID);
}

TEST(Estop, UnknownEndpointIsRejected) {
  spot_sim::EstopSystem system;
  api::EstopCheckInRequest request;
  request.mutable_endpoint()->set_unique_id("nobody");
  EXPECT_EQ(system.CheckIn(request, kT0).status(), api::EstopCheckInResponse::STATUS_ENDPOINT_UNKNOWN);
}
