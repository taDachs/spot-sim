#pragma once

#include <chrono>
#include <cstdint>

#include <google/protobuf/duration.pb.h>
#include <google/protobuf/timestamp.pb.h>

namespace spot_sim {

/// Robot clock time: nanoseconds on the simulator's clock (Gazebo sim time).
using RobotTime = std::chrono::nanoseconds;

/// Wall-clock time. Liveness deadlines (E-Stop check-ins, lease staleness)
/// use it because the SDK's keepalive threads run on wall-clock time, even
/// when the simulation runs faster or slower than real time.
using WallTime = std::chrono::steady_clock::time_point;

inline google::protobuf::Timestamp ToTimestamp(RobotTime t) {
  google::protobuf::Timestamp ts;
  const int64_t ns = t.count();
  int64_t seconds = ns / 1'000'000'000;
  int64_t nanos = ns % 1'000'000'000;
  if (nanos < 0) {
    nanos += 1'000'000'000;
    seconds -= 1;
  }
  ts.set_seconds(seconds);
  ts.set_nanos(static_cast<int32_t>(nanos));
  return ts;
}

inline RobotTime FromTimestamp(const google::protobuf::Timestamp& ts) {
  return RobotTime{ts.seconds() * 1'000'000'000 + ts.nanos()};
}

/// Protobuf Duration: seconds and nanos share the sign.
inline google::protobuf::Duration ToDuration(std::chrono::nanoseconds d) {
  google::protobuf::Duration out;
  const int64_t ns = d.count();
  out.set_seconds(ns / 1'000'000'000);
  out.set_nanos(static_cast<int32_t>(ns % 1'000'000'000));
  return out;
}

inline std::chrono::nanoseconds FromDuration(const google::protobuf::Duration& d) {
  return std::chrono::nanoseconds{d.seconds() * 1'000'000'000 + d.nanos()};
}

}  // namespace spot_sim
