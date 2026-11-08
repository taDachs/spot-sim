#include "spot_sim_core/robot.hpp"

#include <utility>

namespace spot_sim {

Robot::Robot(RobotConfig config) : config_(std::move(config)), snapshot_(SittingSnapshot(RobotTime{0})) {}

void Robot::Step(RobotTime now) {
  now_ns_.store(now.count(), std::memory_order_release);
  std::lock_guard<std::mutex> lock(firmware_mutex_);
  firmware_.power.Step(firmware_.estop.StopLevel(WallNow()), now);
}

void Robot::SetSnapshot(RobotSnapshot snapshot) {
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  snapshot_ = std::move(snapshot);
}

RobotSnapshot Robot::Snapshot() const {
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  return snapshot_;
}

bosdyn::api::RobotState Robot::State() const {
  RobotSnapshot snapshot = Snapshot();
  {
    std::lock_guard<std::mutex> lock(firmware_mutex_);
    snapshot.motor_power = firmware_.power.MotorState();
    for (auto& software_estop : firmware_.estop.States(WallNow())) {
      snapshot.estops.push_back(std::move(software_estop));
    }
  }
  return BuildRobotState(snapshot);
}

}  // namespace spot_sim
