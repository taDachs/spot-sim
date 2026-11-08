#include <gtest/gtest.h>

#include "spot_sim_core/leg_kinematics.hpp"

using spot_sim::LegKinematics;

TEST(LegKinematics, StraightLegReachesFullLength) {
  const auto foot = LegKinematics::ForwardKinematics({0.0, 0.0});
  EXPECT_NEAR(foot.x, 0.025, 1e-12);
  EXPECT_NEAR(foot.z, -0.3205 - 0.3365, 1e-12);
}

TEST(LegKinematics, InverseOfForwardRoundTrips) {
  for (const auto& q : {LegKinematics::Angles{0.8, -1.6}, LegKinematics::Angles{1.2, -2.7},
                        LegKinematics::Angles{0.3, -0.9}, LegKinematics::Angles{-0.2, -1.2}}) {
    const auto foot = LegKinematics::ForwardKinematics(q);
    const auto solved = LegKinematics::InverseKinematics(foot);
    ASSERT_TRUE(solved.has_value());
    EXPECT_NEAR(solved->hip_y, q.hip_y, 1e-9);
    EXPECT_NEAR(solved->knee, q.knee, 1e-9);
  }
}

TEST(LegKinematics, UnreachableTargetHasNoSolution) {
  EXPECT_FALSE(LegKinematics::InverseKinematics({0.0, -1.0}).has_value());
}

TEST(LegKinematics, StandPutsFeetUnderHipsAtNominalHeight) {
  const auto q = LegKinematics::StandAngles(spot_sim::kNominalStandHeight);
  ASSERT_TRUE(q.has_value());
  EXPECT_NEAR(LegKinematics::BodyHeight(*q), spot_sim::kNominalStandHeight, 1e-9);
  EXPECT_NEAR(LegKinematics::ForwardKinematics(*q).x, 0.0, 1e-9);
  // Within the URDF joint limits.
  EXPECT_GT(q->knee, -2.7929);
  EXPECT_LT(q->knee, -0.2548);
}

TEST(LegKinematics, SittingIsLowerThanStanding) {
  const double sit = LegKinematics::BodyHeight(LegKinematics::SitAngles());
  EXPECT_GT(sit, 0.1);
  EXPECT_LT(sit, 0.3);
}

TEST(LegKinematics, MapsUrdfJointNamesToSpotNames) {
  EXPECT_EQ(spot_sim::SpotJointName("front_left_hip_x"), "fl.hx");
  EXPECT_EQ(spot_sim::SpotJointName("rear_right_knee"), "hr.kn");
  EXPECT_FALSE(spot_sim::SpotJointName("arm_sh0").has_value());
}
