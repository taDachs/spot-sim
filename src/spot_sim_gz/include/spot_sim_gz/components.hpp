#pragma once

// Components through which SpotApiSystem (the Spot API "firmware") talks to
// the plugins that move the robot, all attached to the robot's model entity:
//
//   SpotApiSystem --SpotLocomotionCommand--> base plugin
//   SpotApiSystem <--SpotLocomotionStatus--- base plugin
//   SpotApiSystem --SpotArmCommand---------> arm plugin (robots with an arm)
//
// Any base or arm implementation (kinematic, RL policy, ...) only needs this
// header. The arm reports back through its joint positions.

#include <array>
#include <istream>
#include <ostream>

#include <gz/sim/components/Component.hh>
#include <gz/sim/components/Factory.hh>
#include <gz/sim/config.hh>

#include <spot_sim_core/arm.hpp>
#include <spot_sim_core/locomotion.hpp>

namespace spot_sim {
// Stream operators so Gazebo can serialize the components (state logging,
// GUI sync). The format is internal; only round-tripping matters.
inline std::ostream& operator<<(std::ostream& out, const LocomotionCommand& c) {
  return out << c.motors_on << ' ' << static_cast<int>(c.mode) << ' ' << c.sequence << ' ' << c.v_x << ' ' << c.v_y
             << ' ' << c.v_rot << ' ' << c.max_accel_x << ' ' << c.max_accel_y << ' ' << c.max_accel_rot << ' '
             << c.body_height_offset << ' ' << c.body_roll << ' ' << c.body_pitch << ' '
             << c.body_yaw;
}
inline std::istream& operator>>(std::istream& in, LocomotionCommand& c) {
  int mode = 0;
  in >> c.motors_on >> mode >> c.sequence >> c.v_x >> c.v_y >> c.v_rot >> c.max_accel_x >> c.max_accel_y >>
      c.max_accel_rot >> c.body_height_offset >> c.body_roll >>
      c.body_pitch >> c.body_yaw;
  c.mode = static_cast<LocomotionMode>(mode);
  return in;
}
inline std::ostream& operator<<(std::ostream& out, const LocomotionStatus& s) {
  return out << static_cast<int>(s.state) << ' ' << s.reached_sequence;
}
inline std::istream& operator>>(std::istream& in, LocomotionStatus& s) {
  int state = 0;
  in >> state >> s.reached_sequence;
  s.state = static_cast<LocomotionState>(state);
  return in;
}
inline std::ostream& operator<<(std::ostream& out, const ArmCommand& c) {
  out << c.motors_on;
  for (const double q : c.joints.ToArray()) out << ' ' << q;
  return out << ' ' << c.max_joint_speed << ' ' << c.gripper << ' ' << c.max_gripper_speed;
}
inline std::istream& operator>>(std::istream& in, ArmCommand& c) {
  std::array<double, 6> q{};
  in >> c.motors_on;
  for (double& value : q) in >> value;
  c.joints = ArmJoints::FromArray(q);
  return in >> c.max_joint_speed >> c.gripper >> c.max_gripper_speed;
}
}  // namespace spot_sim

namespace gz {
namespace sim {
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace components {

using SpotLocomotionCommand = Component<spot_sim::LocomotionCommand, class SpotLocomotionCommandTag>;
GZ_SIM_REGISTER_COMPONENT("spot_sim.LocomotionCommand", SpotLocomotionCommand)

using SpotLocomotionStatus = Component<spot_sim::LocomotionStatus, class SpotLocomotionStatusTag>;
GZ_SIM_REGISTER_COMPONENT("spot_sim.LocomotionStatus", SpotLocomotionStatus)

using SpotArmCommand = Component<spot_sim::ArmCommand, class SpotArmCommandTag>;
GZ_SIM_REGISTER_COMPONENT("spot_sim.ArmCommand", SpotArmCommand)

}  // namespace components
}  // namespace GZ_SIM_VERSION_NAMESPACE
}  // namespace sim
}  // namespace gz
