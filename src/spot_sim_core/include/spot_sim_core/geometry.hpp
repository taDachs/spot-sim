#pragma once

#include <algorithm>
#include <cmath>

#include <bosdyn/api/geometry.pb.h>

namespace spot_sim {

struct Vec3 {
  double x = 0.0, y = 0.0, z = 0.0;

  Vec3 operator+(const Vec3& o) const { return Vec3{x + o.x, y + o.y, z + o.z}; }
  Vec3 operator-(const Vec3& o) const { return Vec3{x - o.x, y - o.y, z - o.z}; }
  Vec3 operator*(double s) const { return Vec3{x * s, y * s, z * s}; }
  double Dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
  Vec3 Cross(const Vec3& o) const { return Vec3{y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x}; }
  double Norm() const { return std::sqrt(Dot(*this)); }
  Vec3 Normalized() const {
    const double n = Norm();
    return n > 0.0 ? *this * (1.0 / n) : Vec3{};
  }
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

  /// Roll, pitch, yaw with rotation = Rz(yaw) * Ry(pitch) * Rx(roll) (inverse of FromRpy).
  double Roll() const { return std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)); }
  double Pitch() const { return std::asin(std::clamp(2.0 * (w * y - z * x), -1.0, 1.0)); }
  double Yaw() const { return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)); }

  Quat Normalized() const {
    const double n = std::sqrt(w * w + x * x + y * y + z * z);
    return n > 0.0 ? Quat{w / n, x / n, y / n, z / n} : Quat{};
  }
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

  /// Rotation by |v| about v (a rotation vector) [rad].
  static Quat FromRotationVector(const Vec3& v) {
    const double angle = v.Norm();
    if (angle < 1e-12) {
      return Quat{};
    }
    const Vec3 axis = v * (1.0 / angle);
    const double s = std::sin(angle / 2);
    return Quat{std::cos(angle / 2), axis.x * s, axis.y * s, axis.z * s};
  }

  /// Rotation whose matrix has the columns x, y, z (orthonormal, right-handed).
  static Quat FromAxes(const Vec3& x, const Vec3& y, const Vec3& z) {
    const double m00 = x.x, m10 = x.y, m20 = x.z, m01 = y.x, m11 = y.y, m21 = y.z, m02 = z.x, m12 = z.y, m22 = z.z;
    const double trace = m00 + m11 + m22;
    Quat q;
    if (trace > 0.0) {
      const double s = 0.5 / std::sqrt(trace + 1.0);
      q = Quat{0.25 / s, (m21 - m12) * s, (m02 - m20) * s, (m10 - m01) * s};
    } else if (m00 > m11 && m00 > m22) {
      const double s = 2.0 * std::sqrt(1.0 + m00 - m11 - m22);
      q = Quat{(m21 - m12) / s, 0.25 * s, (m01 + m10) / s, (m02 + m20) / s};
    } else if (m11 > m22) {
      const double s = 2.0 * std::sqrt(1.0 + m11 - m00 - m22);
      q = Quat{(m02 - m20) / s, (m01 + m10) / s, 0.25 * s, (m12 + m21) / s};
    } else {
      const double s = 2.0 * std::sqrt(1.0 + m22 - m00 - m11);
      q = Quat{(m10 - m01) / s, (m02 + m20) / s, (m12 + m21) / s, 0.25 * s};
    }
    return q.Normalized();
  }

  double Dot(const Quat& o) const { return w * o.w + x * o.x + y * o.y + z * o.z; }

  /// Angle of the rotation from this orientation to `o` [rad, 0..pi].
  double AngleTo(const Quat& o) const { return 2.0 * std::acos(std::clamp(std::abs(Dot(o)), 0.0, 1.0)); }

  /// Spherical interpolation from this (t = 0) to `o` (t = 1), the short way.
  Quat Slerp(const Quat& o, double t) const {
    Quat b = o;
    double d = Dot(o);
    if (d < 0.0) {
      b = Quat{-o.w, -o.x, -o.y, -o.z};
      d = -d;
    }
    if (d > 0.9995) {
      return Quat{w + t * (b.w - w), x + t * (b.x - x), y + t * (b.y - y), z + t * (b.z - z)}.Normalized();
    }
    const double theta = std::acos(d);
    const double s0 = std::sin((1.0 - t) * theta) / std::sin(theta), s1 = std::sin(t * theta) / std::sin(theta);
    return Quat{s0 * w + s1 * b.w, s0 * x + s1 * b.x, s0 * y + s1 * b.y, s0 * z + s1 * b.z};
  }

  /// Columns of the rotation matrix.
  Vec3 XAxis() const { return Rotate(Vec3{1.0, 0.0, 0.0}); }
  Vec3 YAxis() const { return Rotate(Vec3{0.0, 1.0, 0.0}); }
  Vec3 ZAxis() const { return Rotate(Vec3{0.0, 0.0, 1.0}); }
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

inline Vec3 FromProto(const bosdyn::api::Vec3& v) { return Vec3{v.x(), v.y(), v.z()}; }

/// An unset (all zero) quaternion means no rotation, as in the Spot API.
inline Quat FromProto(const bosdyn::api::Quaternion& q) {
  if (q.w() == 0.0 && q.x() == 0.0 && q.y() == 0.0 && q.z() == 0.0) {
    return Quat{};
  }
  return Quat{q.w(), q.x(), q.y(), q.z()}.Normalized();
}

inline Pose FromProto(const bosdyn::api::SE3Pose& pose) {
  return Pose{FromProto(pose.position()), FromProto(pose.rotation())};
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
