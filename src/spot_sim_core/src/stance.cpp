#include "spot_sim_core/stance.hpp"

#include <algorithm>

namespace spot_sim {

BodyPosture BodyPosture::Clamped() const {
  const BodyPosture lo = Min(), hi = Max();
  return {std::clamp(height_offset, lo.height_offset, hi.height_offset), std::clamp(roll, lo.roll, hi.roll),
          std::clamp(pitch, lo.pitch, hi.pitch), std::clamp(yaw, lo.yaw, hi.yaw)};
}

Pose Stance::BodyInGround(double stand_height, const BodyPosture& posture) {
  return Pose{Vec3{0.0, 0.0, stand_height + posture.height_offset},
              Quat::FromRpy(posture.roll, posture.pitch, posture.yaw)};
}

std::optional<std::array<LegJoints, 4>> Stance::Solve(double stand_height, const BodyPosture& posture,
                                                      const FootOffsets& foot_offsets) {
  const Pose body_T_ground = BodyInGround(stand_height, posture).Inverse();
  std::array<LegJoints, 4> legs;
  for (std::size_t leg = 0; leg < 4; ++leg) {
    const Vec3& offset = foot_offsets[leg];
    // Foot sphere center in the ground frame, then in the body frame.
    const Pose ground_T_foot{Vec3{LegKinematics::SideX(leg) * LegKinematics::kHipX + offset.x,
                                  LegKinematics::SideY(leg) * LegKinematics::kHipY + offset.y,
                                  LegKinematics::kFootRadius + offset.z},
                             Quat{}};
    const Vec3 foot = (body_T_ground * ground_T_foot).position;
    const Vec3 hip = LegKinematics::HipXJoint(leg);
    const auto joints = LegKinematics::InverseLeg(leg, Vec3{foot.x - hip.x, foot.y - hip.y, foot.z - hip.z});
    if (!joints) {
      return std::nullopt;
    }
    legs[leg] = *joints;
  }
  return legs;
}

}  // namespace spot_sim
