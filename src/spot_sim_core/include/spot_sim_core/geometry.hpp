#pragma once

#include <cmath>

#include <bosdyn/api/geometry.pb.h>

namespace spot_sim {

struct Vec3 {
  double x = 0.0, y = 0.0, z = 0.0;
};

struct Quat {
  double w = 1.0, x = 0.0, y = 0.0, z = 0.0;

  static Quat FromYaw(double yaw) { return Quat{std::cos(yaw / 2), 0.0, 0.0, std::sin(yaw / 2)}; }
  static Quat FromRpy(double roll, double pitch, double yaw) {
    const double cr = std::cos(roll / 2), sr = std::sin(roll / 2);
    const double cp = std::cos(pitch / 2), sp = std::sin(pitch / 2);
    const double cy = std::cos(yaw / 2), sy = std::sin(yaw / 2);
    return Quat{cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy};
  }

  double Yaw() const { return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)); }
  Quat Conjugate() const { return Quat{w, -x, -y, -z}; }

  Quat operator*(const Quat& o) const {
    return Quat{w * o.w - x * o.x - y * o.y - z * o.z, w * o.x + x * o.w + y * o.z - z * o.y,
                w * o.y - x * o.z + y * o.w + z * o.x, w * o.z + x * o.y - y * o.x + z * o.w};
  }

  Vec3 Rotate(const Vec3& v) const {
    const Quat p{0.0, v.x, v.y, v.z};
    const Quat r = (*this) * p * Conjugate();
    return Vec3{r.x, r.y, r.z};
  }
};

/// Rigid transform a_T_b: pose of frame b expressed in frame a.
struct Pose {
  Vec3 position;
  Quat rotation;

  Pose operator*(const Pose& o) const {
    const Vec3 p = rotation.Rotate(o.position);
    return Pose{Vec3{position.x + p.x, position.y + p.y, position.z + p.z}, rotation * o.rotation};
  }

  Pose Inverse() const {
    const Quat inv = rotation.Conjugate();
    const Vec3 p = inv.Rotate(position);
    return Pose{Vec3{-p.x, -p.y, -p.z}, inv};
  }
};

inline bosdyn::api::Vec3 ToProto(const Vec3& v) {
  bosdyn::api::Vec3 out;
  out.set_x(v.x);
  out.set_y(v.y);
  out.set_z(v.z);
  return out;
}

inline bosdyn::api::SE3Pose ToProto(const Pose& pose) {
  bosdyn::api::SE3Pose out;
  *out.mutable_position() = ToProto(pose.position);
  out.mutable_rotation()->set_w(pose.rotation.w);
  out.mutable_rotation()->set_x(pose.rotation.x);
  out.mutable_rotation()->set_y(pose.rotation.y);
  out.mutable_rotation()->set_z(pose.rotation.z);
  return out;
}

}  // namespace spot_sim
