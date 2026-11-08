#include "spot_sim_core/robot_frames.hpp"

namespace spot_sim {

std::optional<RobotFrame> ParseRobotFrame(const std::string& name) {
  if (name == kBodyFrame) return RobotFrame::kBody;
  if (name == kFlatBodyFrame) return RobotFrame::kFlatBody;
  if (name == kOdomFrame || name == kVisionFrame || name == kGpeFrame) return RobotFrame::kOdom;
  return std::nullopt;
}

std::optional<RobotFrame> ParsePlanarFrame(const std::string& name) {
  if (name == kGpeFrame) {
    return std::nullopt;
  }
  return ParseRobotFrame(name);
}

Pose BodyTFrame(RobotFrame frame, const Pose& odom_T_body) {
  switch (frame) {
    case RobotFrame::kBody:
      return Pose{};
    case RobotFrame::kFlatBody: {
      const Pose odom_T_flat{odom_T_body.position, Quat::FromYaw(odom_T_body.rotation.Yaw())};
      return odom_T_body.Inverse() * odom_T_flat;
    }
    case RobotFrame::kOdom:
      break;
  }
  return odom_T_body.Inverse();
}

}  // namespace spot_sim
