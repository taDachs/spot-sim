#pragma once

#include <array>
#include <cstddef>
#include <optional>
#include <string>

#include "spot_sim_core/geometry.hpp"

namespace spot_sim {

/// Joint positions of one leg.
struct LegJoints {
  double hip_x = 0.0;
  double hip_y = 0.0;
  double knee = 0.0;
};

/// Planar (sagittal) kinematics of one Spot leg with hip_x = 0, from the
/// spot_description URDF: the knee sits at (0.025, -0.3205) in the upper-leg
/// frame, the foot center at (0, -0.3365) in the lower-leg frame, and the foot
/// is a sphere of radius 0.036. Angles rotate about +y (hip_y, knee); x points
/// forward, z up, both relative to the hip_y joint.
struct LegKinematics {
  static constexpr double kKneeX = 0.025;
  static constexpr double kKneeZ = -0.3205;
  static constexpr double kLowerLegLength = 0.3365;
  static constexpr double kFootRadius = 0.036;

  /// Hip_y joint positions in the body frame (hip_x offset + hip_y offset).
  static constexpr double kHipX = 0.29785;
  static constexpr double kHipXJointY = 0.055;    ///< hip_x joint, lateral
  static constexpr double kHipYOffset = 0.110945;  ///< hip_y joint, lateral from hip_x
  static constexpr double kHipY = kHipXJointY + kHipYOffset;

  struct Point {
    double x = 0.0;
    double z = 0.0;
  };
  struct Angles {
    double hip_y = 0.0;
    double knee = 0.0;
  };

  /// Foot center relative to the hip_y joint.
  static Point ForwardKinematics(const Angles& q);

  /// Joint angles that put the foot center at `foot` (knee bent backwards,
  /// i.e. knee < 0). Empty if unreachable.
  static std::optional<Angles> InverseKinematics(const Point& foot);

  /// Folded legs, body resting low: the sitting pose.
  static Angles SitAngles() { return Angles{1.2, -2.7}; }

  /// Feet directly below the hips with the body origin `body_height` above
  /// the ground.
  static std::optional<Angles> StandAngles(double body_height);

  /// Height of the body origin above flat ground for legs at `q`
  /// (all four legs alike, foot spheres touching the ground).
  static double BodyHeight(const Angles& q) { return -ForwardKinematics(q).z + kFootRadius; }

  // Whole legs, indexed fl, fr, hl, hr (Spot API order).

  /// +1 for front legs, -1 for hind legs.
  static double SideX(std::size_t leg) { return leg < 2 ? 1.0 : -1.0; }
  /// +1 for left legs, -1 for right legs.
  static double SideY(std::size_t leg) { return leg % 2 == 0 ? 1.0 : -1.0; }

  /// The leg's hip_x joint in the body frame.
  static Vec3 HipXJoint(std::size_t leg) { return Vec3{SideX(leg) * kHipX, SideY(leg) * kHipXJointY, 0.0}; }

  /// Foot center relative to the leg's hip_x joint, in the body frame.
  static Vec3 ForwardLeg(std::size_t leg, const LegJoints& q);

  /// Joint positions putting the foot center at `foot_rt_hip_x` (body frame,
  /// relative to the leg's hip_x joint). Empty if unreachable.
  static std::optional<LegJoints> InverseLeg(std::size_t leg, const Vec3& foot_rt_hip_x);
};

/// Nominal standing height of the body origin above the ground [m].
inline constexpr double kNominalStandHeight = 0.52;

/// URDF (spot_description) leg joint name -> Spot API joint name.
std::optional<std::string> SpotJointName(const std::string& urdf_joint_name);

/// URDF leg joint names in Spot API order (fl, fr, hl, hr) x (hx, hy, kn).
const std::array<const char*, 12>& UrdfLegJointNames();

/// URDF lower-leg link names in order fl, fr, hl, hr.
const std::array<const char*, 4>& UrdfLowerLegLinkNames();

}  // namespace spot_sim
