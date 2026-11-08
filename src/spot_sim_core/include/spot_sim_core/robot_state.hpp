#pragma once

#include <array>
#include <optional>
#include <string>
#include <vector>

#include <bosdyn/api/robot_state.pb.h>

#include "spot_sim_core/arm.hpp"
#include "spot_sim_core/geometry.hpp"
#include "spot_sim_core/time.hpp"

namespace spot_sim {

/// Spot API leg joint names, in the order fl, fr, hl, hr x (hx, hy, kn).
inline const std::array<const char*, 12> kLegJointNames = {
    "fl.hx", "fl.hy", "fl.kn", "fr.hx", "fr.hy", "fr.kn", "hl.hx", "hl.hy", "hl.kn", "hr.hx", "hr.hy", "hr.kn"};

struct JointSample {
  std::string name;  ///< Spot API name, e.g. "fl.hx"
  double position = 0.0;
  double velocity = 0.0;
};

/// The arm as the simulation sees it, plus the firmware's grasp bookkeeping.
struct ArmSample {
  ArmJoints joints;
  ArmJoints joint_velocities;
  double gripper = 0.0;  ///< f1x [rad]
  double gripper_velocity = 0.0;
  bool holding_item = false;
  bosdyn::api::ManipulatorState::CarryState carry_state = bosdyn::api::ManipulatorState::CARRY_STATE_NOT_CARRIABLE;
};

struct EstopSample {
  std::string name;
  bosdyn::api::EStopState::Type type = bosdyn::api::EStopState::TYPE_HARDWARE;
  bool estopped = false;
};

/// Everything the simulator knows about the robot at one instant, in plain
/// types. Filled from Gazebo and the firmware state machines; turned into the
/// Spot API RobotState by BuildRobotState().
struct RobotSnapshot {
  RobotTime acquisition_time{0};

  Pose odom_T_body;    ///< kinematic odometry estimate of the body
  Pose vision_T_body;  ///< visual odometry estimate of the body
  Vec3 linear_velocity_in_odom;
  Vec3 angular_velocity_in_odom;

  std::vector<JointSample> joints;
  std::vector<Vec3> foot_positions_rt_body;  ///< fl, fr, hl, hr
  std::vector<bool> foot_contacts;           ///< fl, fr, hl, hr
  std::optional<ArmSample> arm;              ///< robots with an arm only

  bosdyn::api::PowerState::MotorPowerState motor_power = bosdyn::api::PowerState::MOTOR_POWER_STATE_OFF;
  double battery_percentage = 100.0;
  std::vector<EstopSample> estops;
};

/// Snapshot of a powered-off Spot sitting at the odom origin.
RobotSnapshot SittingSnapshot(RobotTime now);

/// Converts a snapshot into the Spot API RobotState, including the frame tree
/// rooted at "body" with "odom", "vision", "flat_body" and "gpe" edges, as a
/// real Spot reports it. With an arm: its joints, ManipulatorState (whose
/// presence tells clients the robot has an arm) and the "hand" frame.
bosdyn::api::RobotState BuildRobotState(const RobotSnapshot& snapshot);

/// Minimal robot metrics.
bosdyn::api::RobotMetrics BuildRobotMetrics(RobotTime now);

}  // namespace spot_sim
