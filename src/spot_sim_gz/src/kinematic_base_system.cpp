// Kinematic floating base for the simulated Spot: the robot's posture and
// pose are set directly every step instead of being simulated by physics.
//
// Contract (see spot_sim_gz/components.hpp): reads SpotLocomotionCommand from
// the model entity, writes SpotLocomotionStatus to it. Body pose, joints and
// feet are read back from Gazebo by SpotApiSystem, so a physics-based base
// (e.g. an RL policy) can replace this plugin without touching the bridge.
//
// Postures: sit <-> stand transitions and collapse on power loss. The legs
// follow joint-space interpolation between the sitting pose and the IK
// solution for the stand height; the body height follows from the legs so the
// feet stay on the ground.
//
// Walking: while standing, the body velocity ramps toward the commanded
// velocity (bounded by the command's acceleration limits) and the planar pose
// is integrated in simulation time. The legs do not step (yet).

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <string>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/AngularVelocityCmd.hh>
#include <gz/sim/components/CollisionBitmask.hh>
#include <gz/sim/components/Gravity.hh>
#include <gz/sim/components/LinearVelocityCmd.hh>
#include <sdf/Element.hh>

#include <spot_sim_core/leg_kinematics.hpp>

#include "spot_sim_gz/components.hpp"

namespace spot_sim_gz {

namespace components = gz::sim::components;
using spot_sim::LegKinematics;
using spot_sim::LocomotionMode;
using spot_sim::LocomotionState;

class KinematicBaseSystem : public gz::sim::System,
                            public gz::sim::ISystemConfigure,
                            public gz::sim::ISystemPreUpdate {
 public:
  void Configure(const gz::sim::Entity& entity, const std::shared_ptr<const sdf::Element>& sdf,
                 gz::sim::EntityComponentManager& ecm, gz::sim::EventManager& /*events*/) override {
    model_ = gz::sim::Model(entity);
    if (!model_.Valid(ecm)) {
      gzerr << "[KinematicBaseSystem] must be attached to a model\n";
      return;
    }
    stand_duration_ = sdf->Get<double>("stand_duration", stand_duration_).first;
    collapse_duration_ = sdf->Get<double>("collapse_duration", collapse_duration_).first;
    const double stand_height = sdf->Get<double>("stand_height", spot_sim::kNominalStandHeight).first;

    const auto stand = LegKinematics::StandAngles(stand_height);
    if (!stand) {
      gzerr << "[KinematicBaseSystem] stand height " << stand_height << " m is not reachable\n";
      return;
    }
    stand_ = *stand;
    sit_ = LegKinematics::SitAngles();

    const auto& names = spot_sim::UrdfLegJointNames();
    for (std::size_t i = 0; i < names.size(); ++i) {
      joints_[i] = gz::sim::Joint(model_.JointByName(ecm, names[i]));
      if (!joints_[i].Valid(ecm)) {
        gzerr << "[KinematicBaseSystem] model has no joint '" << names[i] << "'\n";
        return;
      }
    }

    // This base moves the robot kinematically: keep physics from pulling it
    // down or pushing it around. Only this model is affected.
    for (const auto link : model_.Links(ecm)) {
      ecm.CreateComponent(link, components::GravityEnabledCmd(false));
      ecm.CreateComponent(link, components::GravityEnabled(false));
      for (const auto collision : gz::sim::Link(link).Collisions(ecm)) {
        ecm.CreateComponent(collision, components::CollideBitmaskCmd(0x0));
      }
    }

    const auto pose = gz::sim::worldPose(entity, ecm);
    x_ = pose.Pos().X();
    y_ = pose.Pos().Y();
    yaw_ = pose.Rot().Yaw();
    valid_ = true;
    gzmsg << "[KinematicBaseSystem] kinematic base active on model '" << model_.Name(ecm) << "'\n";
  }

  void PreUpdate(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm) override {
    if (!valid_) {
      return;
    }
    const auto* command_component = ecm.Component<components::SpotLocomotionCommand>(model_.Entity());
    const spot_sim::LocomotionCommand command =
        command_component ? command_component->Data() : spot_sim::LocomotionCommand{};

    if (!info.paused) {
      Advance(command, std::chrono::duration<double>(info.dt).count());
    }
    Apply(ecm);
    ecm.SetComponentData<components::SpotLocomotionStatus>(model_.Entity(), Status(command));
  }

 private:
  /// Moves `value` toward `target` by at most `max_step`.
  static double Approach(double value, double target, double max_step) {
    return value + std::clamp(target - value, -max_step, max_step);
  }

  bool Moving() const {
    constexpr double kEpsilon = 1e-4;
    return std::abs(v_x_) > kEpsilon || std::abs(v_y_) > kEpsilon || std::abs(v_rot_) > kEpsilon;
  }

  void Advance(const spot_sim::LocomotionCommand& command, double dt) {
    AdvancePosture(command, dt);
    AdvanceVelocity(command, dt);
  }

  void AdvanceVelocity(const spot_sim::LocomotionCommand& command, double dt) {
    if (!command.motors_on) {
      v_x_ = v_y_ = v_rot_ = 0.0;
      return;
    }
    // Walk only once fully standing; otherwise come to a stop.
    const bool walking = command.mode == LocomotionMode::kVelocity && progress_ >= 1.0;
    v_x_ = Approach(v_x_, walking ? command.v_x : 0.0, command.max_accel_x * dt);
    v_y_ = Approach(v_y_, walking ? command.v_y : 0.0, command.max_accel_y * dt);
    v_rot_ = Approach(v_rot_, walking ? command.v_rot : 0.0, command.max_accel_rot * dt);

    // Integrate the planar pose (body-frame velocity, midpoint heading).
    const double heading = yaw_ + 0.5 * v_rot_ * dt;
    x_ += (std::cos(heading) * v_x_ - std::sin(heading) * v_y_) * dt;
    y_ += (std::sin(heading) * v_x_ + std::cos(heading) * v_y_) * dt;
    yaw_ = std::remainder(yaw_ + v_rot_ * dt, 2.0 * M_PI);
  }

  void AdvancePosture(const spot_sim::LocomotionCommand& command, double dt) {
    double duration = stand_duration_;
    if (!command.motors_on) {
      standing_target_ = false;
      duration = collapse_duration_;
    } else {
      switch (command.mode) {
        case LocomotionMode::kStand:
        case LocomotionMode::kVelocity:
          standing_target_ = true;
          break;
        case LocomotionMode::kSit:
          standing_target_ = false;
          break;
        case LocomotionMode::kStop:
          break;  // keep the current posture target
      }
    }
    // Come to a stop before sitting down.
    stopping_first_ = command.motors_on && !standing_target_ && Moving();
    if (stopping_first_) {
      standing_target_ = true;
    }
    const double step = dt / std::max(duration, 1e-3);
    progress_ = standing_target_ ? std::min(1.0, progress_ + step) : std::max(0.0, progress_ - step);
  }

  void Apply(gz::sim::EntityComponentManager& ecm) {
    // Smoothstep for a gentle start and stop of the motion.
    const double s = progress_ * progress_ * (3.0 - 2.0 * progress_);
    const LegKinematics::Angles q{sit_.hip_y + s * (stand_.hip_y - sit_.hip_y), sit_.knee + s * (stand_.knee - sit_.knee)};
    const double height = LegKinematics::BodyHeight(q);

    for (std::size_t leg = 0; leg < 4; ++leg) {
      joints_[3 * leg + 0].ResetPosition(ecm, {0.0});
      joints_[3 * leg + 1].ResetPosition(ecm, {q.hip_y});
      joints_[3 * leg + 2].ResetPosition(ecm, {q.knee});
      for (std::size_t j = 0; j < 3; ++j) {
        joints_[3 * leg + j].ResetVelocity(ecm, {0.0});
      }
    }
    model_.SetWorldPoseCmd(ecm, gz::math::Pose3d(x_, y_, height, 0.0, 0.0, yaw_));
    ecm.SetComponentData<components::LinearVelocityCmd>(model_.Entity(), gz::math::Vector3d::Zero);
    ecm.SetComponentData<components::AngularVelocityCmd>(model_.Entity(), gz::math::Vector3d::Zero);
  }

  spot_sim::LocomotionStatus Status(const spot_sim::LocomotionCommand& command) {
    spot_sim::LocomotionStatus status;
    if (progress_ >= 1.0) {
      status.state = Moving() ? LocomotionState::kMoving : LocomotionState::kStanding;
    } else if (progress_ <= 0.0) {
      status.state = LocomotionState::kSitting;
    } else {
      status.state = LocomotionState::kTransitioning;
    }
    const bool at_target = !stopping_first_ && (standing_target_ ? progress_ >= 1.0 : progress_ <= 0.0);
    if (at_target) {
      reached_sequence_ = command.sequence;
    }
    status.reached_sequence = reached_sequence_;
    return status;
  }

  gz::sim::Model model_{gz::sim::kNullEntity};
  std::array<gz::sim::Joint, 12> joints_;
  bool valid_ = false;

  double stand_duration_ = 1.5;     ///< sit <-> stand [s]
  double collapse_duration_ = 0.4;  ///< folding when motor power is lost [s]
  LegKinematics::Angles sit_;
  LegKinematics::Angles stand_;

  double x_ = 0.0, y_ = 0.0, yaw_ = 0.0;
  double v_x_ = 0.0, v_y_ = 0.0, v_rot_ = 0.0;  ///< body-frame velocity
  double progress_ = 0.0;  ///< 0 = sitting, 1 = standing
  bool standing_target_ = false;
  bool stopping_first_ = false;  ///< asked to sit while still walking
  uint32_t reached_sequence_ = 0;
};

}  // namespace spot_sim_gz

GZ_ADD_PLUGIN(spot_sim_gz::KinematicBaseSystem, gz::sim::System, spot_sim_gz::KinematicBaseSystem::ISystemConfigure,
              spot_sim_gz::KinematicBaseSystem::ISystemPreUpdate)
