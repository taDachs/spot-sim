#include "spot_sim_core/api_server.hpp"

#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include <grpcpp/generic/async_generic_service.h>
#include <grpcpp/grpcpp.h>
#include <grpcpp/security/server_credentials.h>
#include <grpcpp/support/server_interceptor.h>

#include <iostream>

#include <bosdyn/api/auth_service.grpc.pb.h>
#include <bosdyn/api/directory_service.grpc.pb.h>
#include <bosdyn/api/estop_service.grpc.pb.h>
#include <bosdyn/api/docking/docking_service.grpc.pb.h>
#include <bosdyn/api/graph_nav/graph_nav_service.grpc.pb.h>
#include <bosdyn/api/image_service.grpc.pb.h>
#include <bosdyn/api/spot/inverse_kinematics_service.grpc.pb.h>
#include <bosdyn/api/lease_service.grpc.pb.h>
#include <bosdyn/api/license_service.grpc.pb.h>
#include <bosdyn/api/power_service.grpc.pb.h>
#include <bosdyn/api/robot_command_service.grpc.pb.h>
#include <bosdyn/api/robot_id_service.grpc.pb.h>
#include <bosdyn/api/robot_state_service.grpc.pb.h>
#include <bosdyn/api/time_sync_service.grpc.pb.h>
#include <bosdyn/api/world_object_service.grpc.pb.h>

#include "spot_sim_core/directory.hpp"
#include "spot_sim_core/response_header.hpp"
#include "spot_sim_core/robot.hpp"

namespace spot_sim {

namespace {

namespace api = bosdyn::api;

std::string ReadFile(const std::filesystem::path& path) {
  std::ifstream in(path);
  if (!in) {
    throw std::runtime_error("cannot read " + path.string());
  }
  std::stringstream buffer;
  buffer << in.rdbuf();
  return buffer.str();
}

/// Runs `body`, then fills the response header with robot-clock timestamps.
template <typename Request, typename Response, typename Body>
grpc::Status Handle(Robot& robot, const Request* request, Response* response, Body&& body) {
  const RobotTime received = robot.Now();
  body();
  FillResponseHeader(*request, response, received, robot.Now());
  return grpc::Status::OK;
}

class AuthServiceImpl final : public api::AuthService::Service {
 public:
  explicit AuthServiceImpl(Robot& robot) : robot_(robot) {}

  grpc::Status GetAuthToken(grpc::ServerContext*, const api::GetAuthTokenRequest* request,
                            api::GetAuthTokenResponse* response) override {
    return Handle(robot_, request, response, [&] {
      const auto& cfg = robot_.config();
      const bool password_ok = request->username() == cfg.username && request->password() == cfg.password;
      const bool token_ok = !request->token().empty() && request->token() == kToken;
      if (password_ok || token_ok) {
        response->set_status(api::GetAuthTokenResponse::STATUS_OK);
        response->set_token(kToken);
      } else if (!request->token().empty()) {
        response->set_status(api::GetAuthTokenResponse::STATUS_INVALID_TOKEN);
      } else {
        response->set_status(api::GetAuthTokenResponse::STATUS_INVALID_LOGIN);
      }
    });
  }

 private:
  static constexpr const char* kToken = "spot-sim-token";
  Robot& robot_;
};

class DirectoryServiceImpl final : public api::DirectoryService::Service {
 public:
  explicit DirectoryServiceImpl(Robot& robot) : robot_(robot), entries_(DefaultServiceEntries()) {}

  grpc::Status GetServiceEntry(grpc::ServerContext*, const api::GetServiceEntryRequest* request,
                               api::GetServiceEntryResponse* response) override {
    return Handle(robot_, request, response, [&] {
      if (const auto entry = FindServiceEntry(entries_, request->service_name())) {
        response->set_status(api::GetServiceEntryResponse::STATUS_OK);
        *response->mutable_service_entry() = *entry;
      } else {
        response->set_status(api::GetServiceEntryResponse::STATUS_NONEXISTENT_SERVICE);
      }
    });
  }

  grpc::Status ListServiceEntries(grpc::ServerContext*, const api::ListServiceEntriesRequest* request,
                                  api::ListServiceEntriesResponse* response) override {
    return Handle(robot_, request, response, [&] {
      for (const auto& entry : entries_) {
        *response->add_service_entries() = entry;
      }
    });
  }

 private:
  Robot& robot_;
  std::vector<api::ServiceEntry> entries_;
};

class RobotIdServiceImpl final : public api::RobotIdService::Service {
 public:
  explicit RobotIdServiceImpl(Robot& robot) : robot_(robot) {}

  grpc::Status GetRobotId(grpc::ServerContext*, const api::RobotIdRequest* request,
                          api::RobotIdResponse* response) override {
    return Handle(robot_, request, response, [&] {
      auto* id = response->mutable_robot_id();
      id->set_serial_number(robot_.config().serial_number);
      id->set_species("spot");
      id->set_version("V3");
      id->set_nickname(robot_.config().nickname);
      id->set_computer_serial_number(robot_.config().serial_number + "-CORE");
      auto* version = id->mutable_software_release()->mutable_version();
      version->set_major_version(5);
      version->set_minor_version(0);
      version->set_patch_level(1);
    });
  }

 private:
  Robot& robot_;
};

class TimeSyncServiceImpl final : public api::TimeSyncService::Service {
 public:
  explicit TimeSyncServiceImpl(Robot& robot) : robot_(robot) {}

  grpc::Status TimeSyncUpdate(grpc::ServerContext*, const api::TimeSyncUpdateRequest* request,
                              api::TimeSyncUpdateResponse* response) override {
    return Handle(robot_, request, response, [&] { *response = robot_.time_sync().Update(*request, robot_.Now()); });
  }

 private:
  Robot& robot_;
};

class LicenseServiceImpl final : public api::LicenseService::Service {
 public:
  explicit LicenseServiceImpl(Robot& robot) : robot_(robot) {}

  grpc::Status GetLicenseInfo(grpc::ServerContext*, const api::GetLicenseInfoRequest* request,
                              api::GetLicenseInfoResponse* response) override {
    return Handle(robot_, request, response, [&] {
      auto* license = response->mutable_license();
      license->set_status(api::LicenseInfo::STATUS_VALID);
      license->set_id("spot-sim-license");
      license->set_robot_serial(robot_.config().serial_number);
      // No optional features (choreography etc.) until they are simulated.
    });
  }

  grpc::Status GetFeatureEnabled(grpc::ServerContext*, const api::GetFeatureEnabledRequest* request,
                                 api::GetFeatureEnabledResponse* response) override {
    return Handle(robot_, request, response, [&] {
      for (const auto& code : request->feature_codes()) {
        (*response->mutable_feature_enabled())[code] = false;
      }
    });
  }

 private:
  Robot& robot_;
};

class RobotStateServiceImpl final : public api::RobotStateService::Service {
 public:
  explicit RobotStateServiceImpl(Robot& robot) : robot_(robot) {}

  grpc::Status GetRobotState(grpc::ServerContext*, const api::RobotStateRequest* request,
                             api::RobotStateResponse* response) override {
    return Handle(robot_, request, response, [&] { *response->mutable_robot_state() = robot_.State(); });
  }

  grpc::Status GetRobotMetrics(grpc::ServerContext*, const api::RobotMetricsRequest* request,
                               api::RobotMetricsResponse* response) override {
    return Handle(robot_, request, response,
                  [&] { *response->mutable_robot_metrics() = BuildRobotMetrics(robot_.Now()); });
  }

  grpc::Status GetRobotHardwareConfiguration(grpc::ServerContext*,
                                             const api::RobotHardwareConfigurationRequest* request,
                                             api::RobotHardwareConfigurationResponse* response) override {
    return Handle(robot_, request, response, [&] { response->mutable_hardware_configuration(); });
  }

 private:
  Robot& robot_;
};

class LeaseServiceImpl final : public api::LeaseService::Service {
 public:
  explicit LeaseServiceImpl(Robot& robot) : robot_(robot) {}

  grpc::Status AcquireLease(grpc::ServerContext*, const api::AcquireLeaseRequest* request,
                            api::AcquireLeaseResponse* response) override {
    return Handle(robot_, request, response, [&] {
      *response = robot_.WithFirmware([&](Firmware& fw) { return fw.lease.Acquire(*request, Robot::WallNow()); });
    });
  }

  grpc::Status TakeLease(grpc::ServerContext*, const api::TakeLeaseRequest* request,
                         api::TakeLeaseResponse* response) override {
    return Handle(robot_, request, response, [&] {
      *response = robot_.WithFirmware([&](Firmware& fw) { return fw.lease.Take(*request, Robot::WallNow()); });
    });
  }

  grpc::Status ReturnLease(grpc::ServerContext*, const api::ReturnLeaseRequest* request,
                           api::ReturnLeaseResponse* response) override {
    return Handle(robot_, request, response, [&] {
      *response = robot_.WithFirmware([&](Firmware& fw) { return fw.lease.Return(*request, Robot::WallNow()); });
    });
  }

  grpc::Status RetainLease(grpc::ServerContext*, const api::RetainLeaseRequest* request,
                           api::RetainLeaseResponse* response) override {
    return Handle(robot_, request, response, [&] {
      *response = robot_.WithFirmware([&](Firmware& fw) { return fw.lease.Retain(*request, Robot::WallNow()); });
    });
  }

  grpc::Status ListLeases(grpc::ServerContext*, const api::ListLeasesRequest* request,
                          api::ListLeasesResponse* response) override {
    return Handle(robot_, request, response, [&] {
      *response = robot_.WithFirmware([&](Firmware& fw) { return fw.lease.List(Robot::WallNow()); });
    });
  }

 private:
  Robot& robot_;
};

class EstopServiceImpl final : public api::EstopService::Service {
 public:
  explicit EstopServiceImpl(Robot& robot) : robot_(robot) {}

  grpc::Status GetEstopConfig(grpc::ServerContext*, const api::GetEstopConfigRequest* request,
                              api::GetEstopConfigResponse* response) override {
    return Handle(robot_, request, response, [&] {
      *response = robot_.WithFirmware([&](Firmware& fw) { return fw.estop.GetConfig(*request); });
    });
  }

  grpc::Status SetEstopConfig(grpc::ServerContext*, const api::SetEstopConfigRequest* request,
                              api::SetEstopConfigResponse* response) override {
    return Handle(robot_, request, response, [&] {
      *response = robot_.WithFirmware(
          [&](Firmware& fw) { return fw.estop.SetConfig(*request, fw.power.MotorsOn()); });
    });
  }

  grpc::Status RegisterEstopEndpoint(grpc::ServerContext*, const api::RegisterEstopEndpointRequest* request,
                                     api::RegisterEstopEndpointResponse* response) override {
    return Handle(robot_, request, response, [&] {
      *response = robot_.WithFirmware([&](Firmware& fw) { return fw.estop.Register(*request); });
    });
  }

  grpc::Status DeregisterEstopEndpoint(grpc::ServerContext*, const api::DeregisterEstopEndpointRequest* request,
                                       api::DeregisterEstopEndpointResponse* response) override {
    return Handle(robot_, request, response, [&] {
      *response = robot_.WithFirmware(
          [&](Firmware& fw) { return fw.estop.Deregister(*request, fw.power.MotorsOn()); });
    });
  }

  grpc::Status EstopCheckIn(grpc::ServerContext*, const api::EstopCheckInRequest* request,
                            api::EstopCheckInResponse* response) override {
    return Handle(robot_, request, response, [&] {
      *response = robot_.WithFirmware([&](Firmware& fw) {
        auto result = fw.estop.CheckIn(*request, Robot::WallNow());
        // Apply a stop immediately rather than at the next simulation step.
        fw.power.Step(fw.estop.StopLevel(Robot::WallNow()), robot_.Now());
        return result;
      });
    });
  }

  grpc::Status GetEstopSystemStatus(grpc::ServerContext*, const api::GetEstopSystemStatusRequest* request,
                                    api::GetEstopSystemStatusResponse* response) override {
    return Handle(robot_, request, response, [&] {
      *response = robot_.WithFirmware([&](Firmware& fw) { return fw.estop.Status(Robot::WallNow()); });
    });
  }

 private:
  Robot& robot_;
};

class RobotCommandServiceImpl final : public api::RobotCommandService::Service {
 public:
  explicit RobotCommandServiceImpl(Robot& robot) : robot_(robot) {}

  grpc::Status RobotCommand(grpc::ServerContext*, const api::RobotCommandRequest* request,
                            api::RobotCommandResponse* response) override {
    return Handle(robot_, request, response, [&] {
      const bool clock_synced = robot_.time_sync().IsSynced(request->clock_identifier());
      const double odom_yaw = robot_.Snapshot().odom_T_body.rotation.Yaw();
      *response = robot_.WithFirmware([&](Firmware& fw) {
        const auto lease = fw.lease.Use(request->lease(), Robot::WallNow());
        return fw.commands.Command(*request, lease, fw.power.MotorsOn(), clock_synced, robot_.Now(), odom_yaw);
      });
    });
  }

  grpc::Status RobotCommandFeedback(grpc::ServerContext*, const api::RobotCommandFeedbackRequest* request,
                                    api::RobotCommandFeedbackResponse* response) override {
    return Handle(robot_, request, response, [&] {
      *response = robot_.WithFirmware([&](Firmware& fw) { return fw.commands.Feedback(*request, fw.locomotion); });
    });
  }

  grpc::Status ClearBehaviorFault(grpc::ServerContext*, const api::ClearBehaviorFaultRequest* request,
                                  api::ClearBehaviorFaultResponse* response) override {
    return Handle(robot_, request, response, [&] {
      *response = robot_.WithFirmware([&](Firmware& fw) {
        api::ClearBehaviorFaultResponse result;
        *result.mutable_lease_use_result() = fw.lease.Use(request->lease(), Robot::WallNow());
        // No behavior faults are simulated, so there is never one to clear.
        result.set_status(api::ClearBehaviorFaultResponse::STATUS_NOT_CLEARED);
        return result;
      });
    });
  }

 private:
  Robot& robot_;
};

class PowerServiceImpl final : public api::PowerService::Service {
 public:
  explicit PowerServiceImpl(Robot& robot) : robot_(robot) {}

  grpc::Status PowerCommand(grpc::ServerContext*, const api::PowerCommandRequest* request,
                            api::PowerCommandResponse* response) override {
    return Handle(robot_, request, response, [&] {
      *response = robot_.WithFirmware([&](Firmware& fw) {
        const WallTime wall = Robot::WallNow();
        const auto lease_result = fw.lease.Use(request->lease(), wall);
        if (lease_result.status() != api::LeaseUseResult::STATUS_OK) {
          api::PowerCommandResponse rejected;
          *rejected.mutable_lease_use_result() = lease_result;
          return rejected;
        }
        auto result = fw.power.Command(*request, fw.estop.StopLevel(wall), robot_.Now());
        *result.mutable_lease_use_result() = lease_result;
        return result;
      });
    });
  }

  grpc::Status PowerCommandFeedback(grpc::ServerContext*, const api::PowerCommandFeedbackRequest* request,
                                    api::PowerCommandFeedbackResponse* response) override {
    return Handle(robot_, request, response, [&] {
      *response = robot_.WithFirmware([&](Firmware& fw) { return fw.power.Feedback(*request); });
    });
  }

 private:
  Robot& robot_;
};

class WorldObjectServiceImpl final : public api::WorldObjectService::Service {
 public:
  explicit WorldObjectServiceImpl(Robot& robot) : robot_(robot) {}

  grpc::Status ListWorldObjects(grpc::ServerContext*, const api::ListWorldObjectRequest* request,
                                api::ListWorldObjectResponse* response) override {
    return Handle(robot_, request, response,
                  [&] { *response = robot_.world_objects().List(*request, robot_.Now()); });
  }

  grpc::Status MutateWorldObjects(grpc::ServerContext*, const api::MutateWorldObjectRequest* request,
                                  api::MutateWorldObjectResponse* response) override {
    return Handle(robot_, request, response,
                  [&] { *response = robot_.world_objects().Mutate(*request, robot_.Now()); });
  }

 private:
  Robot& robot_;
};

/// The robot has no arm, so no inverse kinematics request has a solution.
class InverseKinematicsServiceImpl final : public api::spot::InverseKinematicsService::Service {
 public:
  explicit InverseKinematicsServiceImpl(Robot& robot) : robot_(robot) {}

  grpc::Status InverseKinematics(grpc::ServerContext*, const api::spot::InverseKinematicsRequest* request,
                                 api::spot::InverseKinematicsResponse* response) override {
    return Handle(robot_, request, response, [&] {
      response->set_status(api::spot::InverseKinematicsResponse::STATUS_NO_SOLUTION_FOUND);
      *response->mutable_robot_configuration() = robot_.State().kinematic_state();
    });
  }

 private:
  Robot& robot_;
};

/// No docking station exists in the simulated world: the robot is undocked,
/// docking finds no dock and undocking finds the robot not docked.
class DockingServiceImpl final : public api::docking::DockingService::Service {
 public:
  explicit DockingServiceImpl(Robot& robot) : robot_(robot) {}

  grpc::Status DockingCommand(grpc::ServerContext*, const api::docking::DockingCommandRequest* request,
                              api::docking::DockingCommandResponse* response) override {
    return Handle(robot_, request, response, [&] {
      const auto lease =
          robot_.WithFirmware([&](Firmware& fw) { return fw.lease.Use(request->lease(), Robot::WallNow()); });
      *response->mutable_lease_use_result() = lease;
      if (lease.status() != api::LeaseUseResult::STATUS_OK) {
        response->set_status(api::docking::DockingCommandResponse::STATUS_ERROR_LEASE);
      } else if (request->prep_pose_behavior() == api::docking::PREP_POSE_UNDOCK) {
        response->set_status(api::docking::DockingCommandResponse::STATUS_ERROR_NOT_DOCKED);
      } else {
        response->set_status(api::docking::DockingCommandResponse::STATUS_ERROR_DOCK_NOT_FOUND);
      }
    });
  }

  grpc::Status DockingCommandFeedback(grpc::ServerContext*, const api::docking::DockingCommandFeedbackRequest* request,
                                      api::docking::DockingCommandFeedbackResponse* response) override {
    // No docking command is ever accepted.
    return Handle(robot_, request, response, [&] {
      response->set_status(api::docking::DockingCommandFeedbackResponse::STATUS_OLD_DOCKING_COMMAND);
    });
  }

  grpc::Status GetDockingConfig(grpc::ServerContext*, const api::docking::GetDockingConfigRequest* request,
                                api::docking::GetDockingConfigResponse* response) override {
    return Handle(robot_, request, response, [&] {
      auto* range = response->add_dock_configs();
      range->set_id_start(520);
      range->set_id_end(529);
      range->set_type(api::docking::DOCK_TYPE_SPOT_DOCK);
    });
  }

  grpc::Status GetDockingState(grpc::ServerContext*, const api::docking::GetDockingStateRequest* request,
                               api::docking::GetDockingStateResponse* response) override {
    return Handle(robot_, request, response, [&] {
      response->mutable_dock_state()->set_status(api::docking::DockState::DOCK_STATUS_UNDOCKED);
      response->mutable_dock_state()->set_power_status(api::docking::DockState::LINK_STATUS_UNKNOWN);
    });
  }

 private:
  Robot& robot_;
};

/// GraphNav without maps: the graph is empty and the robot is not localized.
/// Recording, uploading and navigating stay unimplemented.
class GraphNavServiceImpl final : public api::graph_nav::GraphNavService::Service {
 public:
  explicit GraphNavServiceImpl(Robot& robot) : robot_(robot) {}

  grpc::Status GetLocalizationState(grpc::ServerContext*, const api::graph_nav::GetLocalizationStateRequest* request,
                                    api::graph_nav::GetLocalizationStateResponse* response) override {
    return Handle(robot_, request, response, [&] {
      response->mutable_localization();
      *response->mutable_robot_kinematics() = robot_.State().kinematic_state();
    });
  }

  grpc::Status ClearGraph(grpc::ServerContext*, const api::graph_nav::ClearGraphRequest* request,
                          api::graph_nav::ClearGraphResponse* response) override {
    return Handle(robot_, request, response, [&] {
      const auto lease =
          robot_.WithFirmware([&](Firmware& fw) { return fw.lease.Use(request->lease(), Robot::WallNow()); });
      *response->mutable_lease_use_result() = lease;
      if (lease.status() == api::LeaseUseResult::STATUS_OK) {
        response->set_status(api::graph_nav::ClearGraphResponse::STATUS_OK);
      }
    });
  }

  grpc::Status DownloadGraph(grpc::ServerContext*, const api::graph_nav::DownloadGraphRequest* request,
                             api::graph_nav::DownloadGraphResponse* response) override {
    return Handle(robot_, request, response, [&] { response->mutable_graph(); });
  }

 private:
  Robot& robot_;
};

class ImageServiceImpl final : public api::ImageService::Service {
 public:
  explicit ImageServiceImpl(Robot& robot) : robot_(robot) {}

  grpc::Status ListImageSources(grpc::ServerContext*, const api::ListImageSourcesRequest* request,
                                api::ListImageSourcesResponse* response) override {
    return Handle(robot_, request, response, [&] { *response = robot_.cameras().ListImageSources(); });
  }

  grpc::Status GetImage(grpc::ServerContext*, const api::GetImageRequest* request,
                        api::GetImageResponse* response) override {
    return Handle(robot_, request, response, [&] {
      // Image snapshots carry the robot's world frames; the driver publishes
      // every other frame in them as a static transform.
      api::FrameTreeSnapshot tree;
      const auto state = robot_.State();
      for (const auto& [child, edge] : state.kinematic_state().transforms_snapshot().child_to_parent_edge_map()) {
        if (child == "body" || child == "odom" || child == "vision") {
          (*tree.mutable_child_to_parent_edge_map())[child] = edge;
        }
      }
      *response = robot_.cameras().GetImage(*request, tree);
    });
  }

 private:
  Robot& robot_;
};

/// Reports calls to implemented services' unimplemented methods (the
/// generated base classes answer those with UNIMPLEMENTED).
class UnimplementedInterceptor final : public grpc::experimental::Interceptor {
 public:
  UnimplementedInterceptor(grpc::experimental::ServerRpcInfo* info, std::shared_ptr<UnimplementedCalls> calls)
      : method_(info->method()), calls_(std::move(calls)) {}

  void Intercept(grpc::experimental::InterceptorBatchMethods* methods) override {
    if (methods->QueryInterceptionHookPoint(grpc::experimental::InterceptionHookPoints::PRE_SEND_STATUS) &&
        methods->GetSendStatus().error_code() == grpc::StatusCode::UNIMPLEMENTED) {
      calls_->Report(method_);
    }
    methods->Proceed();
  }

 private:
  std::string method_;
  std::shared_ptr<UnimplementedCalls> calls_;
};

class UnimplementedInterceptorFactory final : public grpc::experimental::ServerInterceptorFactoryInterface {
 public:
  explicit UnimplementedInterceptorFactory(std::shared_ptr<UnimplementedCalls> calls) : calls_(std::move(calls)) {}
  grpc::experimental::Interceptor* CreateServerInterceptor(grpc::experimental::ServerRpcInfo* info) override {
    return new UnimplementedInterceptor(info, calls_);
  }

 private:
  std::shared_ptr<UnimplementedCalls> calls_;
};

/// Answers every service the simulator does not implement at all.
class UnknownServices final : public grpc::CallbackGenericService {
 public:
  explicit UnknownServices(std::shared_ptr<UnimplementedCalls> calls) : calls_(std::move(calls)) {}

  grpc::ServerGenericBidiReactor* CreateReactor(grpc::GenericCallbackServerContext* context) override {
    calls_->Report(context->method());
    class Reactor final : public grpc::ServerGenericBidiReactor {
     public:
      Reactor() { Finish(grpc::Status(grpc::StatusCode::UNIMPLEMENTED, "not simulated")); }
      void OnDone() override { delete this; }
    };
    return new Reactor;
  }

 private:
  std::shared_ptr<UnimplementedCalls> calls_;
};

}  // namespace

UnimplementedCalls::UnimplementedCalls(std::string report_file) : report_file_(std::move(report_file)) {
  if (!report_file_.empty()) {
    std::ofstream(report_file_, std::ios::trunc);
  }
}

void UnimplementedCalls::Report(const std::string& method) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (methods_.insert(method).second) {
    std::cerr << "[api] not simulated: " << method << " (answered UNIMPLEMENTED; logged once)" << std::endl;
    if (!report_file_.empty()) {
      std::ofstream(report_file_, std::ios::app) << method << "\n";
    }
  }
}

std::set<std::string> UnimplementedCalls::Methods() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return methods_;
}

TlsConfig TlsConfig::FromDirectory(const std::string& directory) {
  TlsConfig tls;
  const std::filesystem::path dir{directory};
  tls.private_key_pem = ReadFile(dir / "robot.pem");
  for (const auto& entry : std::filesystem::directory_iterator(dir)) {
    const auto name = entry.path().filename().string();
    const std::string suffix = ".spot.robot.crt";
    if (name.size() > suffix.size() && name.compare(name.size() - suffix.size(), suffix.size(), suffix) == 0) {
      tls.certificate_pems.push_back(ReadFile(entry.path()));
    }
  }
  if (tls.certificate_pems.empty()) {
    throw std::runtime_error("no *.spot.robot.crt certificates in " + directory);
  }
  return tls;
}

SpotApiServer::SpotApiServer(Robot& robot, const ServerConfig& config)
    : unimplemented_(std::make_shared<UnimplementedCalls>(config.unimplemented_report_file)) {
  grpc::SslServerCredentialsOptions ssl_options;
  for (const auto& cert : config.tls.certificate_pems) {
    ssl_options.pem_key_cert_pairs.push_back({config.tls.private_key_pem, cert});
  }

  services_.push_back(std::make_unique<AuthServiceImpl>(robot));
  services_.push_back(std::make_unique<DirectoryServiceImpl>(robot));
  services_.push_back(std::make_unique<RobotIdServiceImpl>(robot));
  services_.push_back(std::make_unique<TimeSyncServiceImpl>(robot));
  services_.push_back(std::make_unique<LicenseServiceImpl>(robot));
  services_.push_back(std::make_unique<RobotStateServiceImpl>(robot));
  services_.push_back(std::make_unique<LeaseServiceImpl>(robot));
  services_.push_back(std::make_unique<EstopServiceImpl>(robot));
  services_.push_back(std::make_unique<PowerServiceImpl>(robot));
  services_.push_back(std::make_unique<RobotCommandServiceImpl>(robot));
  services_.push_back(std::make_unique<WorldObjectServiceImpl>(robot));
  services_.push_back(std::make_unique<ImageServiceImpl>(robot));
  services_.push_back(std::make_unique<InverseKinematicsServiceImpl>(robot));
  services_.push_back(std::make_unique<DockingServiceImpl>(robot));
  services_.push_back(std::make_unique<GraphNavServiceImpl>(robot));

  grpc::ServerBuilder builder;
  builder.AddListeningPort(config.address, grpc::SslServerCredentials(ssl_options), &port_);
  for (auto& service : services_) {
    builder.RegisterService(service.get());
  }
  unknown_services_ = std::make_unique<UnknownServices>(unimplemented_);
  builder.RegisterCallbackGenericService(unknown_services_.get());
  std::vector<std::unique_ptr<grpc::experimental::ServerInterceptorFactoryInterface>> interceptors;
  interceptors.push_back(std::make_unique<UnimplementedInterceptorFactory>(unimplemented_));
  builder.experimental().SetInterceptorCreators(std::move(interceptors));
  server_ = builder.BuildAndStart();
  if (!server_) {
    throw std::runtime_error("failed to start Spot API gRPC server on " + config.address);
  }
}

SpotApiServer::~SpotApiServer() { Shutdown(); }

void SpotApiServer::Shutdown() {
  if (server_) {
    server_->Shutdown();
    server_->Wait();
    server_.reset();
  }
}

}  // namespace spot_sim
