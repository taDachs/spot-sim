#include "spot_sim_core/robot.hpp"

#include <utility>

#include "spot_sim_core/arm_kinematics.hpp"

namespace spot_sim {

bosdyn::api::GripperCameraParams DefaultGripperCameraParams() {
  // The hand camera's defaults (slice 10 renders 640x480).
  bosdyn::api::GripperCameraParams params;
  params.set_camera_mode(bosdyn::api::GripperCameraParams::MODE_640_480);
  params.mutable_brightness()->set_value(0.5);
  params.mutable_contrast()->set_value(0.5);
  params.mutable_saturation()->set_value(0.5);
  params.mutable_gain()->set_value(0.5);
  params.mutable_exposure_auto()->set_value(true);
  params.mutable_focus_auto()->set_value(true);
  return params;
}

bosdyn::api::LeaseUseResult Firmware::UseLease(const bosdyn::api::Lease& attempted) {
  return lease.Use(attempted, Robot::WallNow());
}

Robot::Robot(RobotConfig config) : config_(std::move(config)), snapshot_(SittingSnapshot(RobotTime{0})) {
  if (config_.has_arm) {
    snapshot_.arm = ArmSample{};
    snapshot_.arm->joints = ArmKinematics::Stow();
    firmware_.lease.EnableArm();
    firmware_.commands.EnableArm();
    firmware_.gripper_camera_params = DefaultGripperCameraParams();
  }
}

void Robot::Step(RobotTime now) {
  now_ns_.store(now.count(), std::memory_order_release);
  std::optional<ArmSample> arm;
  Pose odom_T_body;
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    arm = snapshot_.arm;
    odom_T_body = snapshot_.odom_T_body;
  }
  std::lock_guard<std::mutex> lock(firmware_mutex_);
  const auto estop_level = firmware_.estop.StopLevel(WallNow());
  firmware_.power.Step(estop_level, now);
  firmware_.commands.Step(now, estop_level, firmware_.power, firmware_.locomotion, arm, odom_T_body);
}

ArmCommand Robot::Arm() {
  std::lock_guard<std::mutex> lock(firmware_mutex_);
  return firmware_.commands.Arm(firmware_.power.MotorsOn());
}

LocomotionCommand Robot::Locomotion() {
  std::lock_guard<std::mutex> lock(firmware_mutex_);
  return firmware_.commands.Locomotion(firmware_.power.MotorsOn());
}

bosdyn::api::LeaseUseResult Robot::UseLease(const bosdyn::api::Lease& lease) {
  std::lock_guard<std::mutex> lock(firmware_mutex_);
  return firmware_.UseLease(lease);
}

void Robot::SetLocomotionStatus(const LocomotionStatus& status) {
  std::lock_guard<std::mutex> lock(firmware_mutex_);
  firmware_.locomotion = status;
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
    if (snapshot.arm) {
      snapshot.arm->holding_item = firmware_.grasp.holding_item;
      snapshot.arm->carry_state = firmware_.grasp.carry_state;
    }
    for (auto& software_estop : firmware_.estop.States(WallNow())) {
      snapshot.estops.push_back(std::move(software_estop));
    }
  }
  return BuildRobotState(snapshot);
}

}  // namespace spot_sim
