#include <gtest/gtest.h>

#include <cmath>

#include "spot_sim_core/gait.hpp"
#include "spot_sim_core/leg_kinematics.hpp"

using spot_sim::LegKinematics;
using spot_sim::TrotGait;
using spot_sim::Vec3;

namespace {
constexpr double kDt = 0.001;
constexpr double kHeight = spot_sim::kNominalStandHeight;

/// Foot center relative to the leg's neutral stance point, from joint angles.
Vec3 ForwardKinematics(std::size_t leg, const spot_sim::LegJoints& q, double body_height) {
  const double y_offset = (leg % 2 == 0 ? 1.0 : -1.0) * LegKinematics::kHipYOffset;
  const auto planar = LegKinematics::ForwardKinematics({q.hip_y, q.knee});
  // Rotate (y_offset, planar.z) about x by hip_x.
  const double y = y_offset * std::cos(q.hip_x) - planar.z * std::sin(q.hip_x);
  const double z = y_offset * std::sin(q.hip_x) + planar.z * std::cos(q.hip_x);
  return Vec3{planar.x, y - y_offset, z + (body_height - LegKinematics::kFootRadius)};
}

void Simulate(TrotGait& gait, double seconds, double v_x, double v_y, double v_rot) {
  for (double t = 0.0; t < seconds; t += kDt) {
    gait.Step(kDt, v_x, v_y, v_rot);
  }
}
}  // namespace

TEST(TrotGait, StandingStillKeepsFeetNeutral) {
  TrotGait gait;
  Simulate(gait, 1.0, 0.0, 0.0, 0.0);
  EXPECT_FALSE(gait.Active());
  for (std::size_t leg = 0; leg < 4; ++leg) {
    const auto offset = gait.FootOffsets()[leg];
    EXPECT_EQ(offset.x, 0.0);
    EXPECT_EQ(offset.z, 0.0);
    const auto q = TrotGait::Solve(leg, offset, kHeight);
    ASSERT_TRUE(q);
    const auto stand = *LegKinematics::StandAngles(kHeight);
    EXPECT_NEAR(q->hip_x, 0.0, 1e-9);
    EXPECT_NEAR(q->hip_y, stand.hip_y, 1e-9);
    EXPECT_NEAR(q->knee, stand.knee, 1e-9);
  }
}

TEST(TrotGait, SolveReachesTheRequestedFootPosition) {
  for (std::size_t leg = 0; leg < 4; ++leg) {
    for (const Vec3 offset : {Vec3{0.15, 0.0, 0.0}, Vec3{-0.1, 0.08, 0.05}, Vec3{0.05, -0.1, 0.08}}) {
      const auto q = TrotGait::Solve(leg, offset, kHeight);
      ASSERT_TRUE(q) << "leg " << leg;
      const auto foot = ForwardKinematics(leg, *q, kHeight);
      EXPECT_NEAR(foot.x, offset.x, 1e-9) << "leg " << leg;
      EXPECT_NEAR(foot.y, offset.y, 1e-9) << "leg " << leg;
      EXPECT_NEAR(foot.z, offset.z, 1e-9) << "leg " << leg;
    }
  }
}

TEST(TrotGait, DiagonalPairsAlternate) {
  TrotGait gait;
  Simulate(gait, 1.0, 0.5, 0.0, 0.0);
  for (int i = 0; i < 20; ++i) {
    Simulate(gait, 0.03, 0.5, 0.0, 0.0);
    EXPECT_EQ(gait.InStance(0), gait.InStance(3));  // fl + hr
    EXPECT_EQ(gait.InStance(1), gait.InStance(2));  // fr + hl
    EXPECT_NE(gait.InStance(0), gait.InStance(1));
  }
}

TEST(TrotGait, StanceFeetStayOnTheGroundWhileWalking) {
  TrotGait gait;
  const double v_x = 0.6, v_y = -0.2;
  Simulate(gait, 1.0, v_x, v_y, 0.0);  // fully blended in
  // Follow fl through one stance phase: in the world it must not move.
  while (gait.InStance(0)) Simulate(gait, kDt, v_x, v_y, 0.0);
  while (!gait.InStance(0)) Simulate(gait, kDt, v_x, v_y, 0.0);
  const Vec3 start = gait.FootOffsets()[0];
  double body_x = 0.0, body_y = 0.0;
  int steps = 0;
  while (gait.InStance(0)) {
    const Vec3 foot = gait.FootOffsets()[0];
    EXPECT_NEAR(body_x + foot.x, start.x, 2e-3);
    EXPECT_NEAR(body_y + foot.y, start.y, 2e-3);
    EXPECT_EQ(foot.z, 0.0);
    Simulate(gait, kDt, v_x, v_y, 0.0);
    body_x += v_x * kDt;
    body_y += v_y * kDt;
    ++steps;
  }
  EXPECT_GT(steps, 200);  // half of the 0.5 s cycle
}

TEST(TrotGait, SwingLiftsTheFoot) {
  TrotGait gait;
  Simulate(gait, 1.0, 0.5, 0.0, 0.0);
  double max_lift = 0.0;
  for (int i = 0; i < 500; ++i) {
    Simulate(gait, kDt, 0.5, 0.0, 0.0);
    max_lift = std::max(max_lift, gait.FootOffsets()[0].z);
  }
  EXPECT_NEAR(max_lift, TrotGait::Params{}.step_height, 1e-3);
}

TEST(TrotGait, StoppingReturnsFeetToNeutral) {
  TrotGait gait;
  Simulate(gait, 1.0, 0.0, 0.0, 0.8);
  EXPECT_TRUE(gait.Active());
  Simulate(gait, 0.5, 0.0, 0.0, 0.0);
  EXPECT_FALSE(gait.Active());
  for (const auto& offset : gait.FootOffsets()) {
    EXPECT_EQ(offset.x, 0.0);
    EXPECT_EQ(offset.y, 0.0);
    EXPECT_EQ(offset.z, 0.0);
  }
}

TEST(TrotGait, FullSpeedStridesAreReachable) {
  TrotGait gait;
  for (int i = 0; i < 2000; ++i) {
    gait.Step(kDt, 1.6, 0.6, 1.2);
    for (std::size_t leg = 0; leg < 4; ++leg) {
      ASSERT_TRUE(TrotGait::Solve(leg, gait.FootOffsets()[leg], kHeight)) << "leg " << leg << " step " << i;
    }
  }
}
