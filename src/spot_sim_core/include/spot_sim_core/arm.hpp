#pragma once

#include <algorithm>
#include <array>
#include <cmath>

namespace spot_sim {

/// Spot Arm joint angles [rad] (the fixed hr0 joint is left out).
struct ArmJoints {
  double sh0 = 0.0, sh1 = 0.0, el0 = 0.0, el1 = 0.0, wr0 = 0.0, wr1 = 0.0;

  std::array<double, 6> ToArray() const { return {sh0, sh1, el0, el1, wr0, wr1}; }
  static ArmJoints FromArray(const std::array<double, 6>& q) { return ArmJoints{q[0], q[1], q[2], q[3], q[4], q[5]}; }

  /// Largest absolute joint difference [rad].
  double MaxDifference(const ArmJoints& o) const {
    const auto a = ToArray(), b = o.ToArray();
    double m = 0.0;
    for (std::size_t i = 0; i < a.size(); ++i) {
      m = std::max(m, std::abs(a[i] - b[i]));
    }
    return m;
  }

  bool operator==(const ArmJoints& o) const { return ToArray() == o.ToArray(); }
  bool operator!=(const ArmJoints& o) const { return !(*this == o); }
};

/// What the robot's "firmware" (the Spot API side) asks the arm plugin to do:
/// move the joints to a setpoint. Plain data, exchanged through a Gazebo
/// component like LocomotionCommand; any arm implementation (kinematic,
/// physics controller, ...) consumes the same struct. The bridge reads the
/// resulting joint positions back from Gazebo.
struct ArmCommand {
  bool motors_on = false;
  ArmJoints joints;              ///< setpoint
  double max_joint_speed = 1.0;  ///< of the fastest-moving joint [rad/s]; the others follow along
  double gripper = 0.0;          ///< f1x setpoint [rad]: 0 closed, -1.57 open
  double max_gripper_speed = 3.0;  ///< [rad/s]

  bool operator==(const ArmCommand& o) const {
    return motors_on == o.motors_on && joints == o.joints && max_joint_speed == o.max_joint_speed &&
           gripper == o.gripper && max_gripper_speed == o.max_gripper_speed;
  }
  bool operator!=(const ArmCommand& o) const { return !(*this == o); }
};

}  // namespace spot_sim
