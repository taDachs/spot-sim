#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <utility>

#include <bosdyn/api/robot_state.pb.h>

#include "spot_sim_core/commands.hpp"
#include "spot_sim_core/estop.hpp"
#include "spot_sim_core/images.hpp"
#include "spot_sim_core/lease.hpp"
#include "spot_sim_core/locomotion.hpp"
#include "spot_sim_core/power.hpp"
#include "spot_sim_core/robot_state.hpp"
#include "spot_sim_core/time.hpp"
#include "spot_sim_core/time_sync.hpp"
#include "spot_sim_core/world_objects.hpp"

namespace spot_sim {

struct RobotConfig {
  std::string nickname = "spot-sim";
  std::string serial_number = "SIM-0000001";
  std::string username = "user";
  std::string password = "password";
};

/// The robot's onboard state machines ("firmware"), guarded by Robot.
struct Firmware {
  LeaseManager lease;
  EstopSystem estop;
  PowerSystem power;
  CommandSystem commands;
  LocomotionStatus locomotion;  ///< last status reported by the base
};

/// The simulated robot's shared state. The simulation thread advances the
/// robot clock and publishes snapshots; gRPC handler threads read them and
/// drive the firmware. All methods are thread-safe.
class Robot {
 public:
  explicit Robot(RobotConfig config);

  const RobotConfig& config() const { return config_; }

  /// Robot clock: the simulation time last published by the simulation.
  RobotTime Now() const { return RobotTime{now_ns_.load(std::memory_order_acquire)}; }

  /// Wall clock used for liveness deadlines.
  static WallTime WallNow() { return std::chrono::steady_clock::now(); }

  /// Called by the simulation every step: advances the robot clock and the
  /// firmware (power transitions, E-Stop timeouts, command sequencing).
  void Step(RobotTime now);

  /// Target for the locomotion base (read by the simulation each step).
  LocomotionCommand Locomotion();

  /// Status reported by the locomotion base (written by the simulation).
  void SetLocomotionStatus(const LocomotionStatus& status);

  /// Replace the simulation-side snapshot (acquisition time set by caller).
  void SetSnapshot(RobotSnapshot snapshot);
  RobotSnapshot Snapshot() const;

  /// RobotState: simulation snapshot plus firmware state (power, E-Stops).
  bosdyn::api::RobotState State() const;

  /// Runs `fn(Firmware&)` under the firmware lock and returns its result.
  template <typename Fn>
  auto WithFirmware(Fn&& fn) {
    std::lock_guard<std::mutex> lock(firmware_mutex_);
    return std::forward<Fn>(fn)(firmware_);
  }

  TimeSyncService& time_sync() { return time_sync_; }

  /// Body cameras (registered and fed by the simulation).
  BodyCameras& cameras() { return cameras_; }

  /// World objects added by clients.
  WorldObjects& world_objects() { return world_objects_; }

 private:
  RobotConfig config_;
  std::atomic<int64_t> now_ns_{0};
  mutable std::mutex snapshot_mutex_;
  RobotSnapshot snapshot_;
  mutable std::mutex firmware_mutex_;
  Firmware firmware_;
  TimeSyncService time_sync_;
  BodyCameras cameras_;
  WorldObjects world_objects_;
};

}  // namespace spot_sim
