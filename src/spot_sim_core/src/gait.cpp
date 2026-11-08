#include "spot_sim_core/gait.hpp"

#include <algorithm>
#include <cmath>

#include "spot_sim_core/leg_kinematics.hpp"

namespace spot_sim {

namespace {
constexpr double kMovingEpsilon = 1e-3;

// Diagonal pairs move together, half a cycle apart.
constexpr std::array<double, 4> kPhaseOffset = {0.0, 0.5, 0.5, 0.0};
}  // namespace

TrotGait::TrotGait(Params params) : params_(params) {}

void TrotGait::Step(double dt, double v_x, double v_y, double v_rot) {
  v_x_ = v_x;
  v_y_ = v_y;
  v_rot_ = v_rot;
  const bool moving =
      std::abs(v_x) > kMovingEpsilon || std::abs(v_y) > kMovingEpsilon || std::abs(v_rot) > kMovingEpsilon;
  const double blend = dt / std::max(params_.blend_time, 1e-3);
  weight_ = std::clamp(weight_ + (moving ? blend : -blend), 0.0, 1.0);
  if (weight_ > 0.0) {
    phase_ = std::fmod(phase_ + dt / params_.period, 1.0);
  }
}

double TrotGait::LegPhase(std::size_t leg) const { return std::fmod(phase_ + kPhaseOffset[leg], 1.0); }

bool TrotGait::InStance(std::size_t leg) const { return weight_ <= 0.0 || LegPhase(leg) < 0.5; }

std::array<Vec3, 4> TrotGait::FootOffsets() const {
  std::array<Vec3, 4> offsets{};
  if (weight_ <= 0.0) {
    return offsets;
  }
  const double stance_time = 0.5 * params_.period;
  for (std::size_t leg = 0; leg < 4; ++leg) {
    // Velocity of the hip over the ground, including the body's rotation.
    const double hip_x = LegKinematics::SideX(leg) * LegKinematics::kHipX;
    const double hip_y = LegKinematics::SideY(leg) * LegKinematics::kHipY;
    const double stride_x = (v_x_ - v_rot_ * hip_y) * stance_time;
    const double stride_y = (v_y_ + v_rot_ * hip_x) * stance_time;

    const double p = LegPhase(leg);
    double along = 0.0;  // fraction of the stride ahead of neutral
    double lift = 0.0;
    if (p < 0.5) {
      along = 0.5 - p / 0.5;  // stance: +1/2 -> -1/2, as fast as the body moves
    } else {
      const double u = (p - 0.5) / 0.5;
      along = -0.5 + 0.5 * (1.0 - std::cos(M_PI * u));  // swing: -1/2 -> +1/2
      lift = params_.step_height * std::sin(M_PI * u);
    }
    offsets[leg] = Vec3{weight_ * along * stride_x, weight_ * along * stride_y, weight_ * lift};
  }
  return offsets;
}

}  // namespace spot_sim
