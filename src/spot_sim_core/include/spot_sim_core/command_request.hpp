#pragma once

// Checks and defaults shared by the RobotCommand request parts (mobility,
// arm, gripper).

#include <string>
#include <variant>

#include <bosdyn/api/robot_command.pb.h>
#include <google/protobuf/timestamp.pb.h>
#include <google/protobuf/wrappers.pb.h>

#include "spot_sim_core/time.hpp"

namespace spot_sim {

/// Why the robot refuses a command: the RobotCommandResponse status and
/// message.
struct Rejection {
  bosdyn::api::RobotCommandResponse::Status status;
  std::string message;
};

/// The end time of a velocity command (`command`: its name, for messages).
/// It is required, needs an established time sync and must not have passed.
std::variant<RobotTime, Rejection> CheckEndTime(const std::string& command, bool has_end_time,
                                                const google::protobuf::Timestamp& end_time, bool clock_synced,
                                                RobotTime now);

/// An optional limit from a request: its value when set and positive,
/// otherwise `fallback`.
inline double PositiveOr(const google::protobuf::DoubleValue& value, double fallback) {
  return value.value() > 0.0 ? value.value() : fallback;
}

}  // namespace spot_sim
