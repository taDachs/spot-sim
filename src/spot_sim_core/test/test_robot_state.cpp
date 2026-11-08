#include <gtest/gtest.h>

#include <set>
#include <string>

#include "spot_sim_core/leg_kinematics.hpp"
#include "spot_sim_core/robot_state.hpp"

using namespace std::chrono_literals;

TEST(RobotState, SittingSnapshotHasAllLegJointsWithSpotNames) {
  const auto state = spot_sim::BuildRobotState(spot_sim::SittingSnapshot(5s));
  std::set<std::string> names;
  for (const auto& joint : state.kinematic_state().joint_states()) {
    names.insert(joint.name());
    EXPECT_TRUE(joint.has_position());
  }
  EXPECT_EQ(names.size(), 12u);
  EXPECT_TRUE(names.count("fl.hx"));
  EXPECT_TRUE(names.count("hr.kn"));
}

TEST(RobotState, FrameTreeIsRootedAtBody) {
  const auto state = spot_sim::BuildRobotState(spot_sim::SittingSnapshot(5s));
  const auto& edges = state.kinematic_state().transforms_snapshot().child_to_parent_edge_map();
  ASSERT_TRUE(edges.count("body"));
  EXPECT_TRUE(edges.at("body").parent_frame_name().empty());
  EXPECT_EQ(edges.at("odom").parent_frame_name(), "body");
  EXPECT_EQ(edges.at("vision").parent_frame_name(), "body");
  EXPECT_EQ(edges.at("flat_body").parent_frame_name(), "body");
  EXPECT_EQ(edges.at("gpe").parent_frame_name(), "odom");
  // The sitting body is above the odom ground plane, so odom is below the body.
  const double sit_height = spot_sim::LegKinematics::BodyHeight(spot_sim::LegKinematics::SitAngles());
  EXPECT_NEAR(edges.at("odom").parent_tform_child().position().z(), -sit_height, 1e-9);
}

TEST(RobotState, CarriesFieldsTheDriverRequires) {
  const auto state = spot_sim::BuildRobotState(spot_sim::SittingSnapshot(5s));
  EXPECT_TRUE(state.has_power_state());
  EXPECT_EQ(state.power_state().motor_power_state(), bosdyn::api::PowerState::MOTOR_POWER_STATE_OFF);
  EXPECT_TRUE(state.has_system_fault_state());
  EXPECT_TRUE(state.has_behavior_fault_state());
  EXPECT_TRUE(state.kinematic_state().has_acquisition_timestamp());
  EXPECT_EQ(state.kinematic_state().acquisition_timestamp().seconds(), 5);
  EXPECT_TRUE(state.kinematic_state().has_velocity_of_body_in_odom());
  EXPECT_TRUE(state.kinematic_state().has_velocity_of_body_in_vision());
  EXPECT_EQ(state.foot_state_size(), 4);
  EXPECT_EQ(state.estop_states_size(), 2);
  // No arm.
  EXPECT_FALSE(state.has_manipulator_state());
}

TEST(Geometry, PoseInverseComposesToIdentity) {
  const spot_sim::Pose a{{1.0, -2.0, 0.5}, spot_sim::Quat::FromRpy(0.1, -0.2, 1.3)};
  const auto identity = a * a.Inverse();
  EXPECT_NEAR(identity.position.x, 0.0, 1e-12);
  EXPECT_NEAR(identity.position.y, 0.0, 1e-12);
  EXPECT_NEAR(identity.position.z, 0.0, 1e-12);
  EXPECT_NEAR(std::abs(identity.rotation.w), 1.0, 1e-12);
  EXPECT_NEAR(spot_sim::Quat::FromYaw(0.7).Yaw(), 0.7, 1e-12);
}
