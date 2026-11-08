// Gazebo system plugin that turns a simulation into a Spot robot on the
// network: it serves the Spot gRPC API (spot_sim_core) and feeds it the
// simulation clock and, in later slices, the robot's state from Gazebo.

#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <sdf/Element.hh>

#include <spot_sim_core/api_server.hpp>
#include <spot_sim_core/robot.hpp>

namespace spot_sim_gz {

class SpotApiSystem : public gz::sim::System, public gz::sim::ISystemConfigure, public gz::sim::ISystemPostUpdate {
 public:
  void Configure(const gz::sim::Entity& /*entity*/, const std::shared_ptr<const sdf::Element>& sdf,
                 gz::sim::EntityComponentManager& /*ecm*/, gz::sim::EventManager& /*events*/) override {
    spot_sim::RobotConfig robot_config;
    robot_config.nickname = sdf->Get<std::string>("nickname", robot_config.nickname).first;
    robot_config.username = sdf->Get<std::string>("username", robot_config.username).first;
    robot_config.password = sdf->Get<std::string>("password", robot_config.password).first;

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
      gzmsg << "[SpotApiSystem] Spot API for '" << robot_config.nickname << "' listening on port " << server_->port()
            << " (certificates from " << certs_dir << ")\n";
    } catch (const std::exception& e) {
      gzerr << "[SpotApiSystem] failed to start Spot API server: " << e.what() << "\n";
    }
  }

  void PostUpdate(const gz::sim::UpdateInfo& info, const gz::sim::EntityComponentManager& /*ecm*/) override {
    if (!robot_) {
      return;
    }
    // The robot clock is simulation time.
    const auto now = std::chrono::duration_cast<spot_sim::RobotTime>(info.simTime);
    snapshot_.acquisition_time = now;
    robot_->SetSnapshot(snapshot_);
    robot_->Step(now);
  }

 private:
  std::unique_ptr<spot_sim::Robot> robot_;
  std::unique_ptr<spot_sim::SpotApiServer> server_;
  spot_sim::RobotSnapshot snapshot_;
};

}  // namespace spot_sim_gz

GZ_ADD_PLUGIN(spot_sim_gz::SpotApiSystem, gz::sim::System, spot_sim_gz::SpotApiSystem::ISystemConfigure,
              spot_sim_gz::SpotApiSystem::ISystemPostUpdate)
