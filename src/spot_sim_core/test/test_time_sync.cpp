#include <gtest/gtest.h>

#include <functional>

#include "spot_sim_core/time_sync.hpp"

using namespace std::chrono_literals;
using spot_sim::FromDuration;
using spot_sim::RobotTime;
using spot_sim::ToTimestamp;
using Status = bosdyn::api::TimeSyncState;

namespace {
bosdyn::api::TimeSyncRoundTrip RoundTrip(RobotTime client_tx, RobotTime server_rx, RobotTime server_tx,
                                         RobotTime client_rx) {
  bosdyn::api::TimeSyncRoundTrip rt;
  *rt.mutable_client_tx() = ToTimestamp(client_tx);
  *rt.mutable_server_rx() = ToTimestamp(server_rx);
  *rt.mutable_server_tx() = ToTimestamp(server_tx);
  *rt.mutable_client_rx() = ToTimestamp(client_rx);
  return rt;
}

/// An exchange with 1 ms each way, robot clock `skew` ahead of the client.
bosdyn::api::TimeSyncRoundTrip Exchange(RobotTime robot_time, std::chrono::nanoseconds skew) {
  const auto client = robot_time - skew;
  return RoundTrip(client - 1ms, robot_time, robot_time, client + 1ms);
}

spot_sim::TimeSyncEstimator::Params Fast() {
  spot_sim::TimeSyncEstimator::Params p;
  p.settle_duration = 0s;
  return p;
}

/// Feed exchanges every 10 ms of robot time from `from` for `duration`.
void Feed(spot_sim::TimeSyncEstimator& estimator, RobotTime from, std::chrono::nanoseconds duration,
          const std::function<std::chrono::nanoseconds(RobotTime)>& skew_at) {
  for (RobotTime t = from; t <= from + duration; t += 10ms) {
    estimator.AddRoundTrip(Exchange(t, skew_at(t)));
  }
}
}  // namespace

TEST(TimeSyncEstimator, NeedsSamplesBeforeOk) {
  spot_sim::TimeSyncEstimator estimator(Fast());
  EXPECT_EQ(estimator.State(0ns).status(), Status::STATUS_MORE_SAMPLES_NEEDED);
  for (int i = 0; i < 3; ++i) {
    estimator.AddRoundTrip(Exchange(1s, 0s));
  }
  EXPECT_EQ(estimator.State(0ns).status(), Status::STATUS_OK);
}

TEST(TimeSyncEstimator, EstimatesSkewAndPicksShortestRoundTrip) {
  spot_sim::TimeSyncEstimator estimator(Fast());
  // Robot clock is 5 s ahead of the client clock.
  estimator.AddRoundTrip(RoundTrip(100s, 105s + 10ms, 105s + 10ms, 100s + 20ms));   // RTT 20 ms, skew 5 s
  estimator.AddRoundTrip(RoundTrip(200s, 205s + 30ms, 205s + 30ms, 200s + 40ms));   // RTT 40 ms, skew 5.01 s
  estimator.AddRoundTrip(RoundTrip(300s, 305s + 15ms, 305s + 15ms, 300s + 30ms));   // RTT 30 ms
  const auto state = estimator.State(0ns);
  ASSERT_EQ(state.status(), Status::STATUS_OK);
  EXPECT_EQ(FromDuration(state.best_estimate().clock_skew()), 5s);
  EXPECT_EQ(FromDuration(state.best_estimate().round_trip_time()), 20ms);
}

TEST(TimeSyncEstimator, ClampsSlightlyNegativeRoundTripFromSteppedSimTime) {
  auto params = Fast();
  params.min_samples = 1;
  spot_sim::TimeSyncEstimator estimator(params);
  estimator.AddRoundTrip(RoundTrip(10s, 10s, 10s + 4ms, 10s));  // RTT -4 ms
  const auto state = estimator.State(0ns);
  ASSERT_EQ(state.status(), Status::STATUS_OK);
  EXPECT_EQ(FromDuration(state.best_estimate().round_trip_time()), 0ns);
}

TEST(TimeSyncEstimator, WaitsForSkewToSettle) {
  spot_sim::TimeSyncEstimator estimator;  // 2 s settle
  Feed(estimator, 10s, 1s, [](RobotTime) { return 0s; });
  EXPECT_FALSE(estimator.IsSynced());
  Feed(estimator, 11s + 10ms, 1s, [](RobotTime) { return 0s; });
  EXPECT_TRUE(estimator.IsSynced());
}

TEST(TimeSyncEstimator, WallClockClientAtRealTimeSyncs) {
  // A driver on wall time (use_sim_time:=false): large but constant skew.
  spot_sim::TimeSyncEstimator estimator;
  const auto wall_offset = std::chrono::seconds(1'794'000'000);
  Feed(estimator, 10s, 3s, [&](RobotTime) { return -wall_offset; });
  ASSERT_TRUE(estimator.IsSynced());
  EXPECT_EQ(FromDuration(estimator.State(13s).best_estimate().clock_skew()), -wall_offset);
}

TEST(TimeSyncEstimator, WallClockClientSyncsAtOtherRealTimeFactors) {
  // Something on wall time while the simulation runs faster or slower than
  // real time, e.g. the driver's launch file querying the robot: the skew
  // drifts steadily. The sync must still establish and track the drift.
  const auto wall_offset = std::chrono::seconds(1'794'000'000);
  for (const double rtf : {0.5, 2.0}) {
    spot_sim::TimeSyncEstimator estimator;
    const auto skew_at = [&](RobotTime t) {
      const auto wall = std::chrono::duration_cast<std::chrono::nanoseconds>(t / rtf) + wall_offset;
      return t - wall;
    };
    Feed(estimator, 10s, 3s, skew_at);
    ASSERT_TRUE(estimator.IsSynced()) << "rtf " << rtf;
    const auto estimate = FromDuration(estimator.State(13s).best_estimate().clock_skew());
    EXPECT_LT(std::chrono::abs(estimate - skew_at(13s)), 20ms) << "rtf " << rtf;
  }
}

TEST(TimeSyncEstimator, UnstartedSimClockSyncsWithZeroSkewAndStaysConsistent) {
  // A ROS node with use_sim_time before its first /clock message reads 0 (the
  // spot_ros2 Python node during its whole startup). It must still sync, and
  // with skew 0, because its clock is about to become simulation time.
  spot_sim::TimeSyncEstimator estimator;
  for (RobotTime t = 10s; t <= 13s; t += 10ms) {
    estimator.AddRoundTrip(RoundTrip(0s, t, t, 0s));
  }
  ASSERT_TRUE(estimator.IsSynced());
  EXPECT_EQ(FromDuration(estimator.State(13s).best_estimate().clock_skew()), 0s);
  // Later, the clock runs on simulation time: no discontinuity.
  estimator.AddRoundTrip(Exchange(70s, 0s));
  EXPECT_TRUE(estimator.IsSynced());
}

TEST(TimeSyncEstimator, DropsSampleWhereClientClockSwitchedMidExchange) {
  // Observed with the driver: client_tx on wall time, client_rx on sim time.
  spot_sim::TimeSyncEstimator estimator;
  Feed(estimator, 10s, 3s, [](RobotTime) { return 0s; });
  ASSERT_TRUE(estimator.IsSynced());
  estimator.AddRoundTrip(RoundTrip(1'794'147'339s, 13s + 100ms, 13s + 100ms, 13s + 100ms));
  EXPECT_TRUE(estimator.IsSynced());
  EXPECT_EQ(FromDuration(estimator.State(14s).best_estimate().clock_skew()), 0s);
}

TEST(TimeSyncEstimator, ResyncsAfterClientClockSwitchesFromWallToSim) {
  spot_sim::TimeSyncEstimator estimator;
  const auto wall_offset = std::chrono::seconds(1'794'000'000);
  Feed(estimator, 10s, 3s, [&](RobotTime) { return -wall_offset; });
  ASSERT_TRUE(estimator.IsSynced());
  // Next periodic update comes from a client now on sim time: discontinuity.
  estimator.AddRoundTrip(Exchange(70s, 0s));
  EXPECT_FALSE(estimator.IsSynced());
  Feed(estimator, 70s + 10ms, 2s, [](RobotTime) { return 0s; });
  ASSERT_TRUE(estimator.IsSynced());
  EXPECT_EQ(FromDuration(estimator.State(73s).best_estimate().clock_skew()), 0s);
}

TEST(TimeSyncService, HandsOutClockIdentifierAndSyncs) {
  spot_sim::TimeSyncService service(Fast());
  bosdyn::api::TimeSyncUpdateRequest request;
  auto response = service.Update(request, 1s);
  const std::string id = response.clock_identifier();
  ASSERT_FALSE(id.empty());
  EXPECT_EQ(response.state().status(), Status::STATUS_MORE_SAMPLES_NEEDED);
  EXPECT_FALSE(service.IsSynced(id));

  request.set_clock_identifier(id);
  for (int i = 0; i < 3; ++i) {
    *request.mutable_previous_round_trip() = RoundTrip(1s, 1s, 1s, 1s + 1ms);
    response = service.Update(request, 2s);
    EXPECT_EQ(response.clock_identifier(), id);
  }
  EXPECT_EQ(response.state().status(), Status::STATUS_OK);
  EXPECT_TRUE(service.IsSynced(id));
  EXPECT_FALSE(service.IsSynced("unknown"));
}

TEST(TimeSyncService, UnknownIdentifierStartsNewClock) {
  spot_sim::TimeSyncService service;
  bosdyn::api::TimeSyncUpdateRequest request;
  request.set_clock_identifier("stale-from-previous-run");
  const auto response = service.Update(request, 1s);
  EXPECT_NE(response.clock_identifier(), "stale-from-previous-run");
}
