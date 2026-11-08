#include "spot_sim_core/leg_kinematics.hpp"

#include <cmath>
#include <map>

#include "spot_sim_core/robot_state.hpp"

namespace spot_sim {

namespace {
// Rotation about +y of a planar (x, z) vector.
LegKinematics::Point Rotate(double q, double x, double z) {
  return {x * std::cos(q) + z * std::sin(q), -x * std::sin(q) + z * std::cos(q)};
}
}  // namespace

LegKinematics::Point LegKinematics::ForwardKinematics(const Angles& q) {
  const Point knee = Rotate(q.hip_y, kKneeX, kKneeZ);
  const Point lower = Rotate(q.hip_y + q.knee, 0.0, -kLowerLegLength);
  return {knee.x + lower.x, knee.z + lower.z};
}

std::optional<LegKinematics::Angles> LegKinematics::InverseKinematics(const Point& foot) {
  // Upper-leg vector a = (kKneeX, kKneeZ) has length l1 and points delta
  // forward of straight down; the lower leg has length l2.
  const double l1 = std::hypot(kKneeX, kKneeZ);
  const double l2 = kLowerLegLength;
  const double delta = std::atan2(kKneeX, -kKneeZ);
  const double d2 = foot.x * foot.x + foot.z * foot.z;
  const double c = (d2 - l1 * l1 - l2 * l2) / (2.0 * l1 * l2);
  if (c < -1.0 || c > 1.0) {
    return std::nullopt;
  }
  // |a + R(knee) b|^2 = l1^2 + l2^2 + 2 l1 l2 cos(knee + delta); knee bends backwards.
  const double knee = -std::acos(c) - delta;
  // In the representation w = z + i x, rotations about +y add to arg(w).
  const Point unrotated = [&] {
    const Point lower = Rotate(knee, 0.0, -kLowerLegLength);
    return Point{kKneeX + lower.x, kKneeZ + lower.z};
  }();
  const double hip_y = std::atan2(foot.x, foot.z) - std::atan2(unrotated.x, unrotated.z);
  return Angles{std::remainder(hip_y, 2.0 * M_PI), knee};
}

std::optional<LegKinematics::Angles> LegKinematics::StandAngles(double body_height) {
  return InverseKinematics(Point{0.0, -(body_height - kFootRadius)});
}

std::optional<std::string> SpotJointName(const std::string& urdf_joint_name) {
  static const std::map<std::string, std::string> kNames = [] {
    std::map<std::string, std::string> names;
    const auto& urdf = UrdfLegJointNames();
    for (std::size_t i = 0; i < urdf.size(); ++i) {
      names.emplace(urdf[i], kLegJointNames[i]);
    }
    return names;
  }();
  const auto it = kNames.find(urdf_joint_name);
  if (it == kNames.end()) {
    return std::nullopt;
  }
  return it->second;
}

const std::array<const char*, 12>& UrdfLegJointNames() {
  static const std::array<const char*, 12> kNames = {
      "front_left_hip_x",  "front_left_hip_y",  "front_left_knee",  "front_right_hip_x",
      "front_right_hip_y", "front_right_knee",  "rear_left_hip_x",  "rear_left_hip_y",
      "rear_left_knee",    "rear_right_hip_x",  "rear_right_hip_y", "rear_right_knee"};
  return kNames;
}

const std::array<const char*, 4>& UrdfLowerLegLinkNames() {
  static const std::array<const char*, 4> kNames = {"front_left_lower_leg", "front_right_lower_leg",
                                                    "rear_left_lower_leg", "rear_right_lower_leg"};
  return kNames;
}

}  // namespace spot_sim
