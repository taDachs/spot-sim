#include "spot_sim_core/arm_kinematics.hpp"

#include <algorithm>
#include <cmath>

namespace spot_sim {

namespace {
Pose Translation(double x, double y, double z) { return Pose{Vec3{x, y, z}, Quat{}}; }
Pose RotX(double a) { return Pose{Vec3{}, Quat::FromRpy(a, 0.0, 0.0)}; }
Pose RotY(double a) { return Pose{Vec3{}, Quat::FromRpy(0.0, a, 0.0)}; }
Pose RotZ(double a) { return Pose{Vec3{}, Quat::FromYaw(a)}; }

/// `angle` shifted by multiples of 2 pi into [lower, upper], as close to
/// `near` as possible; nullopt if no shift fits.
std::optional<double> Fit(double angle, double lower, double upper, double near) {
  constexpr double kSlack = 1e-9;
  std::optional<double> best;
  for (int k = -2; k <= 2; ++k) {
    const double a = angle + 2.0 * M_PI * k;
    if (a < lower - kSlack || a > upper + kSlack) {
      continue;
    }
    if (!best || std::abs(a - near) < std::abs(*best - near)) {
      best = std::clamp(a, lower, upper);
    }
  }
  return best;
}

/// Rotation matrix columns of a quaternion, as m[row][col].
std::array<std::array<double, 3>, 3> Matrix(const Quat& q) {
  const Vec3 x = q.XAxis(), y = q.YAxis(), z = q.ZAxis();
  return {{{x.x, y.x, z.x}, {x.y, y.y, z.y}, {x.z, y.z, z.z}}};
}
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

// The robot's own ready pose (spot_wrapper: "values after unstow").
ArmJoints ArmKinematics::Ready() { return ArmJoints{0.0, -0.9, 1.8, 0.0, -0.9, 0.0}; }

ArmJoints ArmKinematics::Carry() { return ArmJoints{0.0, -2.2, 2.6, 0.0, -0.4, 0.0}; }

Pose ArmKinematics::BodyTWrist(const ArmJoints& q) {
  return Translation(kShoulderX, 0.0, kShoulderZ) * RotZ(q.sh0) * RotY(q.sh1) * Translation(kUpperArm, 0.0, 0.0) *
         RotY(q.el0) * Translation(kForearmX, 0.0, kForearmZ) * RotX(q.el1) * RotY(q.wr0) * RotX(q.wr1);
}

Pose ArmKinematics::BodyTHand(const ArmJoints& q) { return BodyTWrist(q) * Translation(kHandOffset, 0.0, 0.0); }

std::optional<ArmJoints> ArmKinematics::InverseWrist(const Pose& body_T_wrist, const ArmJoints& near) {
  const Vec3 d = body_T_wrist.position - Vec3{kShoulderX, 0.0, kShoulderZ};
  const double forearm = std::hypot(kForearmX, kForearmZ);
  const double forearm_angle = std::atan2(kForearmZ, kForearmX);
  const double reach2 = d.Dot(d);
  // Elbow: |upper arm + forearm| must equal the shoulder-wrist distance.
  const double c = (reach2 - kUpperArm * kUpperArm - forearm * forearm) / (2.0 * kUpperArm * forearm);
  if (c > 1.0 + 1e-9 || c < -1.0 - 1e-9) {
    return std::nullopt;
  }
  const double elbow_spread = std::acos(std::clamp(c, -1.0, 1.0));

  const ArmJoints lower = Lower(), upper = Upper();
  const double planar = std::hypot(d.x, d.y);
  const double yaw = planar < 1e-9 ? near.sh0 : std::atan2(d.y, d.x);

  std::optional<ArmJoints> best;
  double best_distance = 0.0;
  for (const double sh0_raw : {yaw, yaw + M_PI}) {
    const auto sh0 = Fit(sh0_raw, lower.sh0, upper.sh0, near.sh0);
    if (!sh0) continue;
    // The wrist in the arm's vertical plane: u along the turned arm, w up.
    const double u = std::cos(*sh0) * d.x + std::sin(*sh0) * d.y;
    const double w = d.z;
    for (const double el0_raw : {forearm_angle + elbow_spread, forearm_angle - elbow_spread}) {
      const auto el0 = Fit(el0_raw, lower.el0, upper.el0, near.el0);
      if (!el0) continue;
      // Upper arm + forearm before the shoulder pitch; Ry(t) turns an (x, z)
      // vector by -t in the x-z plane.
      const double vx = kUpperArm + kForearmX * std::cos(*el0) + kForearmZ * std::sin(*el0);
      const double vz = -kForearmX * std::sin(*el0) + kForearmZ * std::cos(*el0);
      const auto sh1 = Fit(std::atan2(vz, vx) - std::atan2(w, u), lower.sh1, upper.sh1, near.sh1);
      if (!sh1) continue;

      // Wrist: Rx(el1) Ry(wr0) Rx(wr1) = (Rz(sh0) Ry(sh1 + el0))^T * target.
      const Quat arm = Quat::FromYaw(*sh0) * Quat::FromRpy(0.0, *sh1 + *el0, 0.0);
      const auto m = Matrix(arm.Conjugate() * body_T_wrist.rotation);
      const double wr0_abs = std::acos(std::clamp(m[0][0], -1.0, 1.0));
      for (const double sign : {1.0, -1.0}) {
        double el1 = 0.0, wr0 = sign * wr0_abs, wr1 = 0.0;
        if (std::sin(wr0_abs) < 1e-6) {
          // Wrist straight: only el1 + wr1 matters; keep el1 where it is.
          const double roll = std::atan2(m[2][1], m[1][1]);
          el1 = near.el1;
          wr1 = (m[0][0] > 0.0 ? roll - el1 : -roll + el1);
          wr0 = m[0][0] > 0.0 ? 0.0 : M_PI;
        } else {
          el1 = std::atan2(sign * m[1][0], -sign * m[2][0]);
          wr1 = std::atan2(sign * m[0][1], sign * m[0][2]);
        }
        const auto el1_fit = Fit(el1, lower.el1, upper.el1, near.el1);
        const auto wr0_fit = Fit(wr0, lower.wr0, upper.wr0, near.wr0);
        const auto wr1_fit = Fit(wr1, lower.wr1, upper.wr1, near.wr1);
        if (!el1_fit || !wr0_fit || !wr1_fit) continue;
        const ArmJoints q{*sh0, *sh1, *el0, *el1_fit, *wr0_fit, *wr1_fit};
        // Guard against branch mix-ups: the solution must reproduce the pose.
        const Pose check = BodyTWrist(q);
        if ((check.position - body_T_wrist.position).Norm() > 1e-5 ||
            check.rotation.AngleTo(body_T_wrist.rotation) > 1e-5) {
          continue;
        }
        const double distance = q.MaxDifference(near);
        if (!best || distance < best_distance) {
          best = q;
          best_distance = distance;
        }
      }
    }
  }
  return best;
}

double ArmKinematics::GripperOpenPercentage(double f1x) {
  return std::clamp(f1x / kGripperOpen, 0.0, 1.0) * 100.0;
}

}  // namespace spot_sim
