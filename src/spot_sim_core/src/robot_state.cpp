#include "spot_sim_core/robot_state.hpp"

#include <chrono>

#include "spot_sim_core/leg_kinematics.hpp"

namespace spot_sim {

namespace {
// Sitting pose: legs folded, body resting low on the ground.
const double kSitHipY = LegKinematics::SitAngles().hip_y;
const double kSitKnee = LegKinematics::SitAngles().knee;
const double kSitBodyHeight = LegKinematics::BodyHeight(LegKinematics::SitAngles());
constexpr double kHipX = LegKinematics::kHipX;
constexpr double kHipY = LegKinematics::kHipY;

void SetDouble(google::protobuf::DoubleValue* field, double value) { field->set_value(value); }

void AddEdge(bosdyn::api::FrameTreeSnapshot* tree, const std::string& child, const std::string& parent,
             const Pose& parent_T_child) {
  auto& edge = (*tree->mutable_child_to_parent_edge_map())[child];
  edge.set_parent_frame_name(parent);
  *edge.mutable_parent_tform_child() = ToProto(parent_T_child);
}
}  // namespace

RobotSnapshot SittingSnapshot(RobotTime now) {
  RobotSnapshot s;
  s.acquisition_time = now;
  s.odom_T_body.position.z = kSitBodyHeight;
  s.vision_T_body = s.odom_T_body;
  for (const char* name : kLegJointNames) {
    const std::string n{name};
    double position = 0.0;
    if (n.size() > 3 && n.compare(n.size() - 2, 2, "hy") == 0) position = kSitHipY;
    if (n.size() > 3 && n.compare(n.size() - 2, 2, "kn") == 0) position = kSitKnee;
    s.joints.push_back(JointSample{n, position, 0.0});
  }
  // fl, fr, hl, hr: front legs at +x, left legs at +y.
  constexpr std::array<std::array<double, 2>, 4> kLegSigns = {{{1.0, 1.0}, {1.0, -1.0}, {-1.0, 1.0}, {-1.0, -1.0}}};
  for (const auto& [sx, sy] : kLegSigns) {
    s.foot_positions_rt_body.push_back(Vec3{sx * kHipX, sy * kHipY, -kSitBodyHeight});
  }
  s.foot_contacts.assign(4, true);
  s.estops = {
      EstopSample{"hardware_estop", bosdyn::api::EStopState::TYPE_HARDWARE, false},
      EstopSample{"payload_estop", bosdyn::api::EStopState::TYPE_HARDWARE, false},
  };
  return s;
}

bosdyn::api::RobotState BuildRobotState(const RobotSnapshot& s) {
  bosdyn::api::RobotState state;
  const auto stamp = ToTimestamp(s.acquisition_time);

  // Power.
  auto* power = state.mutable_power_state();
  *power->mutable_timestamp() = stamp;
  power->set_motor_power_state(s.motor_power);
  power->set_shore_power_state(bosdyn::api::PowerState::SHORE_POWER_STATE_OFF);
  power->set_robot_power_state(bosdyn::api::PowerState::ROBOT_POWER_STATE_ON);
  power->set_payload_ports_power_state(bosdyn::api::PowerState::PAYLOAD_PORTS_POWER_STATE_ON);
  power->set_wifi_radio_power_state(bosdyn::api::PowerState::WIFI_RADIO_POWER_STATE_ON);
  SetDouble(power->mutable_locomotion_charge_percentage(), s.battery_percentage);
  *power->mutable_locomotion_estimated_runtime() = ToDuration(std::chrono::minutes(90));

  // Battery.
  auto* battery = state.add_battery_states();
  *battery->mutable_timestamp() = stamp;
  battery->set_identifier("sim-battery");
  SetDouble(battery->mutable_charge_percentage(), s.battery_percentage);
  *battery->mutable_estimated_runtime() = ToDuration(std::chrono::minutes(90));
  SetDouble(battery->mutable_current(), -2.0);
  SetDouble(battery->mutable_voltage(), 56.0);
  battery->add_temperatures(30.0);
  battery->set_status(bosdyn::api::BatteryState::STATUS_DISCHARGING);

  // E-Stops.
  for (const auto& e : s.estops) {
    auto* estop = state.add_estop_states();
    *estop->mutable_timestamp() = stamp;
    estop->set_name(e.name);
    estop->set_type(e.type);
    estop->set_state(e.estopped ? bosdyn::api::EStopState::STATE_ESTOPPED : bosdyn::api::EStopState::STATE_NOT_ESTOPPED);
  }

  // Fault states: present but empty, as on a healthy robot.
  state.mutable_system_fault_state();
  state.mutable_behavior_fault_state();
  state.mutable_service_fault_state();

  // Kinematics.
  auto* kin = state.mutable_kinematic_state();
  *kin->mutable_acquisition_timestamp() = stamp;
  for (const auto& j : s.joints) {
    auto* joint = kin->add_joint_states();
    joint->set_name(j.name);
    SetDouble(joint->mutable_position(), j.position);
    SetDouble(joint->mutable_velocity(), j.velocity);
    SetDouble(joint->mutable_acceleration(), 0.0);
    SetDouble(joint->mutable_load(), 0.0);
  }

  // Frame tree rooted at "body", like the real robot.
  auto* tree = kin->mutable_transforms_snapshot();
  (*tree->mutable_child_to_parent_edge_map())["body"];  // root: no parent
  AddEdge(tree, "odom", "body", s.odom_T_body.Inverse());
  AddEdge(tree, "vision", "body", s.vision_T_body.Inverse());
  const Pose odom_T_flat_body{s.odom_T_body.position, Quat::FromYaw(s.odom_T_body.rotation.Yaw())};
  AddEdge(tree, "flat_body", "body", s.odom_T_body.Inverse() * odom_T_flat_body);
  // Ground plane estimate: the odom ground plane (flat ground for now).
  AddEdge(tree, "gpe", "odom", Pose{});

  // Body velocity (same estimate in both frames until vision drift exists).
  const Quat vision_R_odom = s.vision_T_body.rotation * s.odom_T_body.rotation.Conjugate();
  auto* v_odom = kin->mutable_velocity_of_body_in_odom();
  *v_odom->mutable_linear() = ToProto(s.linear_velocity_in_odom);
  *v_odom->mutable_angular() = ToProto(s.angular_velocity_in_odom);
  auto* v_vision = kin->mutable_velocity_of_body_in_vision();
  *v_vision->mutable_linear() = ToProto(vision_R_odom.Rotate(s.linear_velocity_in_odom));
  *v_vision->mutable_angular() = ToProto(vision_R_odom.Rotate(s.angular_velocity_in_odom));

  // Feet.
  for (std::size_t i = 0; i < s.foot_positions_rt_body.size(); ++i) {
    auto* foot = state.add_foot_state();
    *foot->mutable_foot_position_rt_body() = ToProto(s.foot_positions_rt_body[i]);
    const bool contact = i < s.foot_contacts.size() && s.foot_contacts[i];
    foot->set_contact(contact ? bosdyn::api::FootState::CONTACT_MADE : bosdyn::api::FootState::CONTACT_LOST);
  }

  // No manipulator_state: the Python driver infers "has arm" from its presence.
  return state;
}

bosdyn::api::RobotMetrics BuildRobotMetrics(RobotTime now) {
  bosdyn::api::RobotMetrics metrics;
  *metrics.mutable_timestamp() = ToTimestamp(now);
  return metrics;
}

}  // namespace spot_sim
