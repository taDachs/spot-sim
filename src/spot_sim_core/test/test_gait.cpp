#include <gtest/gtest.h>

#include <cmath>

#include "spot_sim_core/gait.hpp"
#include "spot_sim_core/leg_kinematics.hpp"
#include "spot_sim_core/stance.hpp"

using spot_sim::LegKinematics;
using spot_sim::TrotGait;
using spot_sim::Vec3;

namespace {
constexpr double kDt = 0.001;
constexpr double kHeight = spot_sim::kNominalStandHeight;

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
    EXPECT_EQ(offset.y, 0.0);
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
    ASSERT_TRUE(spot_sim::Stance::Solve(kHeight, {}, gait.FootOffsets())) << "step " << i;
  }
}
