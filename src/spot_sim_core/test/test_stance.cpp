#include <gtest/gtest.h>

#include "spot_sim_core/stance.hpp"

using spot_sim::BodyPosture;
using spot_sim::LegJoints;
using spot_sim::LegKinematics;
using spot_sim::Pose;
using spot_sim::Quat;
using spot_sim::Stance;
using spot_sim::Vec3;

namespace {
constexpr double kHeight = spot_sim::kNominalStandHeight;

/// Foot sphere center in the ground frame, from joint positions.
Vec3 FootInGround(std::size_t leg, const LegJoints& q, double stand_height, const BodyPosture& posture) {
  const Vec3 hip = LegKinematics::HipXJoint(leg);
  const Vec3 rt_hip = LegKinematics::ForwardLeg(leg, q);
  const Pose body_T_foot{Vec3{hip.x + rt_hip.x, hip.y + rt_hip.y, hip.z + rt_hip.z}, Quat{}};
  return (Stance::BodyInGround(stand_height, posture) * body_T_foot).position;
}

Vec3 NeutralFoot(std::size_t leg) {
  return Vec3{LegKinematics::SideX(leg) * LegKinematics::kHipX, LegKinematics::SideY(leg) * LegKinematics::kHipY,
              LegKinematics::kFootRadius};
}
}  // namespace

TEST(LegKinematics, InverseLegOfForwardLegRoundTrips) {
  for (std::size_t leg = 0; leg < 4; ++leg) {
    for (const LegJoints q : {LegJoints{0.0, 0.8, -1.6}, LegJoints{0.2, 0.5, -1.2}, LegJoints{-0.3, 1.0, -2.0}}) {
      const auto solved = LegKinematics::InverseLeg(leg, LegKinematics::ForwardLeg(leg, q));
      ASSERT_TRUE(solved) << "leg " << leg;
      EXPECT_NEAR(solved->hip_x, q.hip_x, 1e-9) << "leg " << leg;
      EXPECT_NEAR(solved->hip_y, q.hip_y, 1e-9) << "leg " << leg;
      EXPECT_NEAR(solved->knee, q.knee, 1e-9) << "leg " << leg;
    }
  }
}

TEST(Stance, NeutralPostureIsTheNominalStand) {
  const auto legs = Stance::Solve(kHeight, {});
  ASSERT_TRUE(legs);
  const auto stand = *LegKinematics::StandAngles(kHeight);
  for (const auto& q : *legs) {
    EXPECT_NEAR(q.hip_x, 0.0, 1e-9);
    EXPECT_NEAR(q.hip_y, stand.hip_y, 1e-9);
    EXPECT_NEAR(q.knee, stand.knee, 1e-9);
  }
}

TEST(Stance, FeetStayPutWhateverTheBodyPosture) {
  for (const BodyPosture posture :
       {BodyPosture{-0.15, 0.0, 0.0, 0.0}, BodyPosture{0.1, 0.0, 0.0, 0.0}, BodyPosture{0.0, 0.3, 0.0, 0.0},
        BodyPosture{0.0, 0.0, -0.3, 0.0}, BodyPosture{0.0, 0.0, 0.0, 0.4}, BodyPosture{-0.1, 0.2, 0.25, -0.3}}) {
    const Stance::FootOffsets offsets = {Vec3{0.05, 0.0, 0.0}, Vec3{}, Vec3{-0.05, 0.03, 0.06}, Vec3{}};
    const auto legs = Stance::Solve(kHeight, posture, offsets);
    ASSERT_TRUE(legs) << posture.height_offset << " " << posture.roll << " " << posture.pitch << " " << posture.yaw;
    for (std::size_t leg = 0; leg < 4; ++leg) {
      const Vec3 foot = FootInGround(leg, (*legs)[leg], kHeight, posture);
      const Vec3 expected = NeutralFoot(leg);
      EXPECT_NEAR(foot.x, expected.x + offsets[leg].x, 1e-9) << "leg " << leg;
      EXPECT_NEAR(foot.y, expected.y + offsets[leg].y, 1e-9) << "leg " << leg;
      EXPECT_NEAR(foot.z, expected.z + offsets[leg].z, 1e-9) << "leg " << leg;
    }
  }
}

TEST(Stance, BodyPoseFollowsThePosture) {
  const auto body = Stance::BodyInGround(kHeight, BodyPosture{-0.1, 0.1, 0.2, 0.3});
  EXPECT_NEAR(body.position.z, kHeight - 0.1, 1e-12);
  EXPECT_NEAR(body.rotation.Roll(), 0.1, 1e-12);
  EXPECT_NEAR(body.rotation.Pitch(), 0.2, 1e-12);
  EXPECT_NEAR(body.rotation.Yaw(), 0.3, 1e-12);
}

TEST(Stance, OutOfReachPostureHasNoSolution) {
  EXPECT_FALSE(Stance::Solve(kHeight, BodyPosture{0.4, 0.0, 0.0, 0.0}));
}

TEST(Stance, PostureRequestsAreSaturated) {
  const auto clamped = BodyPosture{1.0, -1.0, 0.1, 2.0}.Clamped();
  EXPECT_EQ(clamped.height_offset, BodyPosture::Max().height_offset);
  EXPECT_EQ(clamped.roll, BodyPosture::Min().roll);
  EXPECT_EQ(clamped.pitch, 0.1);
  EXPECT_EQ(clamped.yaw, BodyPosture::Max().yaw);
  // The documented stand height range is reachable.
  EXPECT_TRUE(Stance::Solve(kHeight, BodyPosture{BodyPosture::Min().height_offset, 0.0, 0.0, 0.0}));
  EXPECT_TRUE(Stance::Solve(kHeight, BodyPosture{BodyPosture::Max().height_offset, 0.0, 0.0, 0.0}));
}
