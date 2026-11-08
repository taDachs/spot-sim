#include "spot_sim_core/time_sync.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>

namespace spot_sim {

namespace {
std::chrono::nanoseconds Abs(std::chrono::nanoseconds d) { return d < std::chrono::nanoseconds{0} ? -d : d; }

bool DebugEnabled() {
  const char* value = std::getenv("SPOT_SIM_DEBUG_TIMESYNC");
  return value != nullptr && value[0] != '\0' && value[0] != '0';
}
}  // namespace

TimeSyncEstimator::TimeSyncEstimator(Params params) : params_(params) {}

void TimeSyncEstimator::Reset() {
  samples_.clear();
  run_count_ = 0;
  established_ = false;
}

void TimeSyncEstimator::AddRoundTrip(const bosdyn::api::TimeSyncRoundTrip& rt) {
  if (!rt.has_client_tx() || !rt.has_server_rx() || !rt.has_server_tx() || !rt.has_client_rx()) {
    return;
  }
  const auto client_tx = FromTimestamp(rt.client_tx());
  const auto server_rx = FromTimestamp(rt.server_rx());
  const auto server_tx = FromTimestamp(rt.server_tx());
  const auto client_rx = FromTimestamp(rt.client_rx());

  auto round_trip = (client_rx - client_tx) - (server_tx - server_rx);
  // A client clock that jumped during the exchange (e.g. wall -> sim time)
  // produces an absurd round trip. Drop such samples.
  if (round_trip < -params_.max_negative_round_trip || round_trip > params_.max_round_trip) {
    return;
  }
  // Stepped simulation clocks can make the round trip slightly negative.
  round_trip = std::max(round_trip, std::chrono::nanoseconds{0});
  auto skew = ((server_rx - client_tx) + (server_tx - client_rx)) / 2;
  // A ROS node with use_sim_time reads exactly 0 until its first /clock
  // message. Its clock will become simulation time, i.e. the robot clock, so
  // the right skew for it is 0 (not "robot time - 0", which would latch a
  // wrong offset until the client's next periodic update, a minute later).
  // The spot_ros2 Python node sits in this state for its whole startup.
  if (client_tx >= std::chrono::nanoseconds{0} && client_tx < params_.unstarted_clock_threshold &&
      client_rx >= std::chrono::nanoseconds{0} && client_rx < params_.unstarted_clock_threshold) {
    skew = std::chrono::nanoseconds{0};
  }

  // Compare with the previous sample rather than the start of the run: a
  // client on wall time sees a steadily drifting skew whenever the
  // simulation does not run at real time, which is fine; a jump is not.
  const bool consistent = run_count_ > 0 && Abs(skew - last_skew_) <= params_.tolerance;
  if (!consistent) {
    // First sample, or a discontinuity (e.g. the client clock switched from
    // wall to simulation time): start a new consistent run.
    Reset();
    run_start_ = server_rx;
  }
  last_skew_ = skew;
  ++run_count_;
  samples_.push_back(Sample{round_trip, skew});
  while (samples_.size() > params_.window) {
    samples_.pop_front();
  }
  if (run_count_ >= params_.min_samples && server_rx - run_start_ >= params_.settle_duration) {
    established_ = true;
  }
}

bosdyn::api::TimeSyncState TimeSyncEstimator::State(RobotTime now) const {
  bosdyn::api::TimeSyncState state;
  *state.mutable_measurement_time() = ToTimestamp(now);
  if (!established_ || samples_.empty()) {
    state.set_status(bosdyn::api::TimeSyncState::STATUS_MORE_SAMPLES_NEEDED);
    return state;
  }
  // Best estimate: the sample with the shortest round trip has the least
  // asymmetric delay. Among equals take the newest, which tracks a drifting
  // skew best.
  auto best = samples_.begin();
  for (auto it = samples_.begin(); it != samples_.end(); ++it) {
    if (it->round_trip <= best->round_trip) {
      best = it;
    }
  }
  *state.mutable_best_estimate()->mutable_round_trip_time() = ToDuration(best->round_trip);
  *state.mutable_best_estimate()->mutable_clock_skew() = ToDuration(best->skew);
  state.set_status(bosdyn::api::TimeSyncState::STATUS_OK);
  return state;
}

bosdyn::api::TimeSyncUpdateResponse TimeSyncService::Update(const bosdyn::api::TimeSyncUpdateRequest& request,
                                                            RobotTime now) {
  std::lock_guard<std::mutex> lock(mutex_);
  bosdyn::api::TimeSyncUpdateResponse response;

  std::string id = request.clock_identifier();
  auto it = clocks_.find(id);
  if (id.empty() || it == clocks_.end()) {
    id = "spot-sim-clock-" + std::to_string(next_id_++);
    it = clocks_.emplace(id, TimeSyncEstimator{params_}).first;
  } else if (request.has_previous_round_trip()) {
    if (DebugEnabled()) {
      const auto& rt = request.previous_round_trip();
      const auto sec = [](const google::protobuf::Timestamp& t) { return t.seconds() + t.nanos() * 1e-9; };
      std::fprintf(stderr, "[timesync] %s client_tx=%.3f server_rx=%.3f server_tx=%.3f client_rx=%.3f\n", id.c_str(),
                   sec(rt.client_tx()), sec(rt.server_rx()), sec(rt.server_tx()), sec(rt.client_rx()));
    }
    const auto before = it->second.State(now);
    if (before.status() == bosdyn::api::TimeSyncState::STATUS_OK) {
      *response.mutable_previous_estimate() = before.best_estimate();
    }
    it->second.AddRoundTrip(request.previous_round_trip());
  }

  *response.mutable_state() = it->second.State(now);
  response.set_clock_identifier(id);
  return response;
}

bool TimeSyncService::IsSynced(const std::string& clock_identifier) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = clocks_.find(clock_identifier);
  return it != clocks_.end() && it->second.IsSynced();
}

}  // namespace spot_sim
