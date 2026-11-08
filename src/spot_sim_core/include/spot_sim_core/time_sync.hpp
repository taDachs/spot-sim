#pragma once

#include <chrono>
#include <cstddef>
#include <deque>
#include <map>
#include <mutex>
#include <string>

#include <bosdyn/api/time_sync.pb.h>

#include "spot_sim_core/time.hpp"

namespace spot_sim {

/// NTP-style clock skew estimation for one client clock, as the robot's
/// TimeSyncService does it. The client reports the four timestamps of its
/// previous exchange; the robot derives round-trip time and skew
/// (skew = robot clock - client clock).
///
/// Simulation-specific robustness: ROS nodes with use_sim_time start their
/// clock at 0 or on wall time and switch to simulation time once /clock
/// arrives, often after they have started time sync. The estimator therefore
///  - drops samples whose round trip is implausible (the client clock jumped
///    mid-exchange),
///  - treats a client clock that has not started (reads ~0: use_sim_time
///    without /clock yet) as simulation time, i.e. skew 0,
///  - reports OK only after the skew has been stable (within `tolerance`) for
///    `settle_duration` of robot time, so a clock that switches from wall to
///    simulation time right after startup is caught before sync is declared,
///  - restarts estimation when a later sample's skew jumps (discontinuity).
class TimeSyncEstimator {
 public:
  struct Params {
    std::size_t min_samples = 3;
    std::chrono::nanoseconds settle_duration = std::chrono::seconds(2);
    std::chrono::nanoseconds tolerance = std::chrono::milliseconds(250);
    std::chrono::nanoseconds max_negative_round_trip = std::chrono::milliseconds(50);
    std::chrono::nanoseconds max_round_trip = std::chrono::seconds(10);
    std::chrono::nanoseconds unstarted_clock_threshold = std::chrono::seconds(1);
    std::size_t window = 32;
  };

  TimeSyncEstimator() : TimeSyncEstimator(Params{}) {}
  explicit TimeSyncEstimator(Params params);

  /// Add a completed round trip.
  void AddRoundTrip(const bosdyn::api::TimeSyncRoundTrip& round_trip);

  /// Current state, stamped with `now` (robot clock).
  bosdyn::api::TimeSyncState State(RobotTime now) const;

  bool IsSynced() const { return established_; }

 private:
  struct Sample {
    std::chrono::nanoseconds round_trip;
    std::chrono::nanoseconds skew;
  };

  void Reset();

  Params params_;
  std::deque<Sample> samples_;        ///< recent consistent samples (best estimate pool)
  RobotTime run_start_{0};            ///< robot time of the first sample of the consistent run
  std::chrono::nanoseconds run_skew_{0};  ///< skew at the start of the consistent run
  std::size_t run_count_ = 0;
  bool established_ = false;
};

/// Holds one estimator per clock identifier handed out to clients.
class TimeSyncService {
 public:
  explicit TimeSyncService(TimeSyncEstimator::Params params = {}) : params_(params) {}

  /// Handles one TimeSyncUpdate. `now` is robot-clock time. The response
  /// header is left for the caller to fill.
  bosdyn::api::TimeSyncUpdateResponse Update(const bosdyn::api::TimeSyncUpdateRequest& request, RobotTime now);

  /// True if `clock_identifier` was handed out and has an established sync.
  bool IsSynced(const std::string& clock_identifier) const;

 private:
  TimeSyncEstimator::Params params_;
  mutable std::mutex mutex_;
  std::map<std::string, TimeSyncEstimator> clocks_;
  int next_id_ = 0;
};

}  // namespace spot_sim
