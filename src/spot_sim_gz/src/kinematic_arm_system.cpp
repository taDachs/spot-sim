// Kinematic Spot Arm: the arm's joints are set directly every step instead of
// being driven by physics.
//
// Contract (see spot_sim_gz/components.hpp): reads SpotArmCommand from the
// model entity and moves the arm and gripper joints toward its setpoints.
// SpotApiSystem reads the joint positions back from Gazebo, so a physics arm
// controller can replace this plugin without touching the bridge.
//
// The arm moves in a straight line in joint space: the joint with the
// farthest to go moves at the commanded speed, the others scaled so that all
// arrive together. Without motor power the arm stays where it is. It boots
// stowed with the gripper closed.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <sdf/Element.hh>

#include <spot_sim_core/arm_kinematics.hpp>

#include "spot_sim_gz/components.hpp"

namespace spot_sim_gz {

namespace components = gz::sim::components;
using spot_sim::ArmKinematics;

class KinematicArmSystem : public gz::sim::System, public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate {
 public:
  void Configure(const gz::sim::Entity& entity, const std::shared_ptr<const sdf::Element>& /*sdf*/,
                 gz::sim::EntityComponentManager& ecm, gz::sim::EventManager& /*events*/) override {
    model_ = gz::sim::Model(entity);
    if (!model_.Valid(ecm)) {
      gzerr << "[KinematicArmSystem] must be attached to a model\n";
      return;
    }
    for (std::size_t i = 0; i < joints_.size(); ++i) {
      joints_[i] = gz::sim::Joint(model_.JointByName(ecm, spot_sim::kUrdfArmJointNames[i]));
    }
    gripper_joint_ = gz::sim::Joint(model_.JointByName(ecm, spot_sim::kUrdfGripperJointName));
    for (const auto& joint : joints_) {
      if (!joint.Valid(ecm)) {
        gzerr << "[KinematicArmSystem] model has no arm joints (" << spot_sim::kUrdfArmJointNames[0] << ", ...)\n";
        return;
      }
    }
    if (!gripper_joint_.Valid(ecm)) {
      gzerr << "[KinematicArmSystem] model has no gripper joint '" << spot_sim::kUrdfGripperJointName << "'\n";
      return;
    }
    joints_q_ = ArmKinematics::Stow().ToArray();
    gripper_q_ = ArmKinematics::kGripperClosed;
    valid_ = true;
    gzmsg << "[KinematicArmSystem] kinematic arm active on model '" << model_.Name(ecm) << "'\n";
  }

  void PreUpdate(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm) override {
    if (!valid_) {
      return;
    }
    if (!info.paused) {
      if (const auto* command = ecm.Component<components::SpotArmCommand>(model_.Entity());
          command && command->Data().motors_on) {
        Advance(command->Data(), std::chrono::duration<double>(info.dt).count());
      }
    }
    for (std::size_t i = 0; i < joints_.size(); ++i) {
      joints_[i].ResetPosition(ecm, {joints_q_[i]});
      joints_[i].ResetVelocity(ecm, {0.0});
    }
    gripper_joint_.ResetPosition(ecm, {gripper_q_});
    gripper_joint_.ResetVelocity(ecm, {0.0});
  }

 private:
  void Advance(const spot_sim::ArmCommand& command, double dt) {
    const auto target = ArmKinematics::Clamped(command.joints).ToArray();
    double farthest = 0.0;
    for (std::size_t i = 0; i < target.size(); ++i) {
      farthest = std::max(farthest, std::abs(target[i] - joints_q_[i]));
    }
    const double step = std::max(command.max_joint_speed, 0.0) * dt;
    if (farthest <= step) {
      joints_q_ = target;
    } else {
      const double fraction = step / farthest;
      for (std::size_t i = 0; i < target.size(); ++i) {
        joints_q_[i] += (target[i] - joints_q_[i]) * fraction;
      }
    }
    const double gripper_target = ArmKinematics::ClampedGripper(command.gripper);
    const double gripper_step = std::max(command.max_gripper_speed, 0.0) * dt;
    gripper_q_ += std::clamp(gripper_target - gripper_q_, -gripper_step, gripper_step);
  }

  gz::sim::Model model_{gz::sim::kNullEntity};
  std::array<gz::sim::Joint, 6> joints_;
  gz::sim::Joint gripper_joint_;
  bool valid_ = false;
  std::array<double, 6> joints_q_{};
  double gripper_q_ = 0.0;
};

}  // namespace spot_sim_gz

GZ_ADD_PLUGIN(spot_sim_gz::KinematicArmSystem, gz::sim::System, spot_sim_gz::KinematicArmSystem::ISystemConfigure,
              spot_sim_gz::KinematicArmSystem::ISystemPreUpdate)
