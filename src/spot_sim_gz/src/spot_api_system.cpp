// Gazebo system plugin that turns a simulation into a Spot robot on the
// network: it serves the Spot gRPC API (spot_sim_core), feeds it the
// simulation clock and the robot's state as Gazebo sees it, and hands the
// resulting locomotion targets to the base plugin on the robot model.

#include <array>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <optional>
#include <string>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <sdf/Element.hh>

#include <spot_sim_core/api_server.hpp>
#include <spot_sim_core/leg_kinematics.hpp>
#include <spot_sim_core/robot.hpp>

#include "spot_sim_gz/components.hpp"

namespace spot_sim_gz {

namespace components = gz::sim::components;

namespace {
spot_sim::Pose ToPose(const gz::math::Pose3d& p) {
  return spot_sim::Pose{{p.Pos().X(), p.Pos().Y(), p.Pos().Z()},
                        {p.Rot().W(), p.Rot().X(), p.Rot().Y(), p.Rot().Z()}};
}
}  // namespace

class SpotApiSystem : public gz::sim::System,
                      public gz::sim::ISystemConfigure,
                      public gz::sim::ISystemPreUpdate,
                      public gz::sim::ISystemPostUpdate {
 public:
  void Configure(const gz::sim::Entity& /*entity*/, const std::shared_ptr<const sdf::Element>& sdf,
                 gz::sim::EntityComponentManager& /*ecm*/, gz::sim::EventManager& /*events*/) override {
    spot_sim::RobotConfig robot_config;
    robot_config.nickname = sdf->Get<std::string>("nickname", robot_config.nickname).first;
    robot_config.username = sdf->Get<std::string>("username", robot_config.username).first;
    robot_config.password = sdf->Get<std::string>("password", robot_config.password).first;
    model_name_ = sdf->Get<std::string>("model_name", model_name_).first;
    body_link_name_ = sdf->Get<std::string>("body_link", body_link_name_).first;

    const char* env_certs = std::getenv("SPOT_SIM_CERTS_DIR");
    const std::string certs_dir = sdf->Get<std::string>("certs_dir", env_certs ? env_certs : "/certs").first;
    const int port = sdf->Get<int>("port", 443).first;

    robot_ = std::make_unique<spot_sim::Robot>(robot_config);
    snapshot_ = spot_sim::SittingSnapshot(spot_sim::RobotTime{0});

    try {
      spot_sim::ServerConfig server_config;
      server_config.address = "0.0.0.0:" + std::to_string(port);
      server_config.tls = spot_sim::TlsConfig::FromDirectory(certs_dir);
      server_ = std::make_unique<spot_sim::SpotApiServer>(*robot_, server_config);
      gzmsg << "[SpotApiSystem] Spot API for '" << robot_config.nickname << "' (model '" << model_name_
            << "') listening on port " << server_->port() << " (certificates from " << certs_dir << ")\n";
    } catch (const std::exception& e) {
      gzerr << "[SpotApiSystem] failed to start Spot API server: " << e.what() << "\n";
    }
  }

  void PreUpdate(const gz::sim::UpdateInfo& /*info*/, gz::sim::EntityComponentManager& ecm) override {
    if (!robot_) {
      return;
    }
    if (!model_) {
      FindModel(ecm);
    }
    if (model_) {
      ecm.SetComponentData<components::SpotLocomotionCommand>(model_->Entity(), robot_->Locomotion());
    }
  }

  void PostUpdate(const gz::sim::UpdateInfo& info, const gz::sim::EntityComponentManager& ecm) override {
    if (!robot_) {
      return;
    }
    // The robot clock is simulation time.
    const auto now = std::chrono::duration_cast<spot_sim::RobotTime>(info.simTime);
    if (model_) {
      ReadSnapshot(ecm, now);
      if (const auto* status = ecm.Component<components::SpotLocomotionStatus>(model_->Entity())) {
        robot_->SetLocomotionStatus(status->Data());
      }
    }
    snapshot_.acquisition_time = now;
    robot_->SetSnapshot(snapshot_);
    robot_->Step(now);
  }

 private:
  void FindModel(gz::sim::EntityComponentManager& ecm) {
    const auto entity = ecm.EntityByComponents(components::Model(), components::Name(model_name_));
    if (entity == gz::sim::kNullEntity) {
      return;
    }
    gz::sim::Model model(entity);
    body_link_ = model.LinkByName(ecm, body_link_name_);
    if (body_link_ == gz::sim::kNullEntity) {
      gzerr << "[SpotApiSystem] model '" << model_name_ << "' has no link '" << body_link_name_ << "'\n";
      return;
    }
    const auto& joint_names = spot_sim::UrdfLegJointNames();
    for (std::size_t i = 0; i < joint_names.size(); ++i) {
      joints_[i] = gz::sim::Joint(model.JointByName(ecm, joint_names[i]));
      joints_[i].EnablePositionCheck(ecm, true);
      joints_[i].EnableVelocityCheck(ecm, true);
    }
    const auto& leg_names = spot_sim::UrdfLowerLegLinkNames();
    for (std::size_t i = 0; i < leg_names.size(); ++i) {
      lower_legs_[i] = model.LinkByName(ecm, leg_names[i]);
    }
    model_ = model;
    gzmsg << "[SpotApiSystem] found robot model '" << model_name_ << "'\n";
  }

  void ReadSnapshot(const gz::sim::EntityComponentManager& ecm, spot_sim::RobotTime now) {
    const auto world_T_body = gz::sim::worldPose(body_link_, ecm);
    if (!world_T_odom_) {
      // Odometry starts where the robot boots: on the ground below the body,
      // facing the body's heading.
      world_T_odom_ = gz::math::Pose3d(world_T_body.Pos().X(), world_T_body.Pos().Y(), 0.0, 0.0, 0.0,
                                       world_T_body.Rot().Yaw());
    }
    const auto odom_T_body = world_T_odom_->Inverse() * world_T_body;
    snapshot_.odom_T_body = ToPose(odom_T_body);
    snapshot_.vision_T_body = snapshot_.odom_T_body;
    UpdateVelocity(odom_T_body, now);

    snapshot_.joints.clear();
    for (std::size_t i = 0; i < joints_.size(); ++i) {
      const auto position = joints_[i].Position(ecm);
      const auto velocity = joints_[i].Velocity(ecm);
      snapshot_.joints.push_back(spot_sim::JointSample{
          spot_sim::kLegJointNames[i], position && !position->empty() ? position->front() : 0.0,
          velocity && !velocity->empty() ? velocity->front() : 0.0});
    }

    snapshot_.foot_positions_rt_body.clear();
    snapshot_.foot_contacts.clear();
    const auto body_T_world = world_T_body.Inverse();
    for (const auto link : lower_legs_) {
      const auto world_T_leg = gz::sim::worldPose(link, ecm);
      const auto foot_world =
          world_T_leg.Pos() + world_T_leg.Rot().RotateVector({0.0, 0.0, -spot_sim::LegKinematics::kLowerLegLength});
      const auto foot_body = body_T_world.Pos() + body_T_world.Rot().RotateVector(foot_world);
      snapshot_.foot_positions_rt_body.push_back(spot_sim::Vec3{foot_body.X(), foot_body.Y(), foot_body.Z()});
      // Flat ground at z = 0 for now; contact when the foot sphere touches it.
      snapshot_.foot_contacts.push_back(foot_world.Z() - spot_sim::LegKinematics::kFootRadius < 0.02);
    }
  }

  /// Body velocity in odom by differencing poses over simulation time, so it
  /// works for any base (kinematic or physics-driven).
  void UpdateVelocity(const gz::math::Pose3d& odom_T_body, spot_sim::RobotTime now) {
    if (previous_pose_ && now > previous_time_) {
      const double dt = std::chrono::duration<double>(now - previous_time_).count();
      const auto linear = (odom_T_body.Pos() - previous_pose_->Pos()) / dt;
      // Rotation from the previous to the current orientation, in odom.
      gz::math::Vector3d axis;
      double angle = 0.0;
      (odom_T_body.Rot() * previous_pose_->Rot().Inverse()).AxisAngle(axis, angle);
      if (angle > GZ_PI) {
        angle -= 2.0 * GZ_PI;  // q and -q are the same rotation: take the short way
      }
      const auto angular = axis * (angle / dt);
      snapshot_.linear_velocity_in_odom = spot_sim::Vec3{linear.X(), linear.Y(), linear.Z()};
      snapshot_.angular_velocity_in_odom = spot_sim::Vec3{angular.X(), angular.Y(), angular.Z()};
    }
    if (!previous_pose_ || now != previous_time_) {
      previous_pose_ = odom_T_body;
      previous_time_ = now;
    }
  }

  std::unique_ptr<spot_sim::Robot> robot_;
  std::unique_ptr<spot_sim::SpotApiServer> server_;
  spot_sim::RobotSnapshot snapshot_;

  std::string model_name_ = "spot";
  std::string body_link_name_ = "body";
  std::optional<gz::sim::Model> model_;
  gz::sim::Entity body_link_ = gz::sim::kNullEntity;
  std::array<gz::sim::Joint, 12> joints_;
  std::array<gz::sim::Entity, 4> lower_legs_{};
  std::optional<gz::math::Pose3d> world_T_odom_;
  std::optional<gz::math::Pose3d> previous_pose_;
  spot_sim::RobotTime previous_time_{0};
};

}  // namespace spot_sim_gz

GZ_ADD_PLUGIN(spot_sim_gz::SpotApiSystem, gz::sim::System, spot_sim_gz::SpotApiSystem::ISystemConfigure,
              spot_sim_gz::SpotApiSystem::ISystemPreUpdate, spot_sim_gz::SpotApiSystem::ISystemPostUpdate)
