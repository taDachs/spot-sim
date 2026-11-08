#pragma once

#include <array>

#include "spot_sim_core/arm.hpp"
#include "spot_sim_core/geometry.hpp"

namespace spot_sim {

/// Spot API joint names of the arm, as the robot reports them in its joint
/// states (sh0 .. wr1, the fixed hr0 among them, and the gripper f1x).
inline const std::array<const char*, 8> kArmJointNames = {"arm0.sh0", "arm0.sh1", "arm0.hr0", "arm0.el0",
                                                          "arm0.el1", "arm0.wr0", "arm0.wr1", "arm0.f1x"};

/// The arm's joints in spot_description's URDF (and so in Gazebo), in
/// ArmJoints order, and the gripper joint.
inline const std::array<const char*, 6> kUrdfArmJointNames = {"arm_sh0", "arm_sh1", "arm_el0",
                                                              "arm_el1", "arm_wr0", "arm_wr1"};
inline constexpr const char* kUrdfGripperJointName = "arm_f1x";

/// Kinematics of the Spot Arm, from spot_description's arm and gripper URDF:
///   sh0  yaw   at body (0.292, 0, 0.188)
///   sh1  pitch at the same point
///   el0  pitch 0.3385 along the upper arm
///   el1  roll  at (0.4033, 0, 0.075) from el0
///   wr0  pitch, wr1 roll: same point as el1 (a spherical wrist)
///   hand frame 0.19557 along x of wr1 (the Spot API "hand" frame)
///   f1x  gripper jaw, 0 closed .. -1.57 open
struct ArmKinematics {
  static constexpr double kShoulderX = 0.292, kShoulderZ = 0.188;
  static constexpr double kUpperArm = 0.3385;
  static constexpr double kForearmX = 0.4033, kForearmZ = 0.075;
  static constexpr double kHandOffset = 0.19557;
  static constexpr double kGripperOpen = -1.57, kGripperClosed = 0.0;

  /// Joint limits (URDF).
  static ArmJoints Lower();
  static ArmJoints Upper();
  static ArmJoints Clamped(const ArmJoints& q);
  static double ClampedGripper(double q) { return std::clamp(q, kGripperOpen, kGripperClosed); }

  /// Named arm positions (NamedArmPositionsCommand). Approximations of the
  /// robot's: stow folds the arm onto the top of the body, hand forward;
  /// ready holds the hand forward, level, in front of the body; carry holds
  /// it closer in, above the front of the body.
  static ArmJoints Stow();
  static ArmJoints Ready();
  static ArmJoints Carry();

  /// Pose of the wrist (link_wr1) and of the hand frame in the body frame.
  static Pose BodyTWrist(const ArmJoints& q);
  static Pose BodyTHand(const ArmJoints& q);

  /// Gripper opening in percent (0 closed, 100 open), as ManipulatorState reports it.
  static double GripperOpenPercentage(double f1x);
};

}  // namespace spot_sim
