#include "spot_sim_core/arm_kinematics.hpp"

#include <algorithm>
#include <cmath>

namespace spot_sim {

namespace {
Pose Translation(double x, double y, double z) { return Pose{Vec3{x, y, z}, Quat{}}; }
Pose RotX(double a) { return Pose{Vec3{}, Quat::FromRpy(a, 0.0, 0.0)}; }
Pose RotY(double a) { return Pose{Vec3{}, Quat::FromRpy(0.0, a, 0.0)}; }
Pose RotZ(double a) { return Pose{Vec3{}, Quat::FromYaw(a)}; }
}  // namespace

ArmJoints ArmKinematics::Lower() { return ArmJoints{-2.617993877991494, -M_PI, 0.0, -2.792526803190927, -1.832595714594046, -2.879793265790644}; }

ArmJoints ArmKinematics::Upper() { return ArmJoints{M_PI, 0.523598775598299, M_PI, 2.792526803190927, 1.832595714594046, 2.879793265790644}; }

ArmJoints ArmKinematics::Clamped(const ArmJoints& q) {
  const auto value = q.ToArray(), lower = Lower().ToArray(), upper = Upper().ToArray();
  std::array<double, 6> out{};
  for (std::size_t i = 0; i < out.size(); ++i) {
    out[i] = std::clamp(value[i], lower[i], upper[i]);
  }
  return ArmJoints::FromArray(out);
}

// Upper arm folded back (sh1 ~ -pi), forearm folded forward over it
// (el0 ~ pi): the wrist rests above the front half of the body, hand forward.
ArmJoints ArmKinematics::Stow() { return ArmJoints{0.0, -3.11, 3.13, 0.0, -0.02, 0.0}; }

// Upper arm raised forward, forearm angled down; the wrist pitch levels the hand.
ArmJoints ArmKinematics::Ready() { return ArmJoints{0.0, -1.3, 2.2, 0.0, -0.9, 0.0}; }

ArmJoints ArmKinematics::Carry() { return ArmJoints{0.0, -2.2, 2.6, 0.0, -0.4, 0.0}; }

Pose ArmKinematics::BodyTWrist(const ArmJoints& q) {
  return Translation(kShoulderX, 0.0, kShoulderZ) * RotZ(q.sh0) * RotY(q.sh1) * Translation(kUpperArm, 0.0, 0.0) *
         RotY(q.el0) * Translation(kForearmX, 0.0, kForearmZ) * RotX(q.el1) * RotY(q.wr0) * RotX(q.wr1);
}

Pose ArmKinematics::BodyTHand(const ArmJoints& q) { return BodyTWrist(q) * Translation(kHandOffset, 0.0, 0.0); }

double ArmKinematics::GripperOpenPercentage(double f1x) {
  return std::clamp(f1x / kGripperOpen, 0.0, 1.0) * 100.0;
}

}  // namespace spot_sim
