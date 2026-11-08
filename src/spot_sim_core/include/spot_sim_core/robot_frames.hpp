#pragma once

#include <optional>
#include <string>

#include "spot_sim_core/geometry.hpp"

namespace spot_sim {

/// Names of the robot's frames, as in its frame tree snapshots.
inline constexpr const char* kBodyFrame = "body";
inline constexpr const char* kFlatBodyFrame = "flat_body";
inline constexpr const char* kOdomFrame = "odom";
inline constexpr const char* kVisionFrame = "vision";
inline constexpr const char* kGpeFrame = "gpe";
inline constexpr const char* kHandFrame = "hand";

/// Robot frames commands may be expressed in. "vision" and "gpe" are the
/// same as "odom" in the simulator (no drift, flat ground).
enum class RobotFrame { kBody, kFlatBody, kOdom };

/// The names ParseRobotFrame accepts, for error messages.
inline constexpr const char* kRobotFrameNames = "body, flat_body, odom, vision or gpe";
std::optional<RobotFrame> ParseRobotFrame(const std::string& name);

/// Frames a planar (se2) velocity may be given in: the robot frames except
/// "gpe", which is not gravity-aligned on the robot. ("body" is accepted as
/// flat_body: the SDK's velocity command builder sends it.)
inline constexpr const char* kPlanarFrameNames = "body, flat_body, odom or vision";
std::optional<RobotFrame> ParsePlanarFrame(const std::string& name);

/// body_T_frame for the robot frames above.
Pose BodyTFrame(RobotFrame frame, const Pose& odom_T_body);

}  // namespace spot_sim
