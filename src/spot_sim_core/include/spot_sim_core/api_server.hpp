#pragma once

#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

namespace grpc {
class CallbackGenericService;
class Server;
class Service;
}  // namespace grpc

namespace spot_sim {

class Robot;

struct TlsConfig {
  std::string private_key_pem;
  /// One certificate per authority (api/auth/id.spot.robot), all for the same
  /// key. The server picks one by the client's SNI.
  std::vector<std::string> certificate_pems;

  /// Loads `robot.pem` and every `*.spot.robot.crt` from `directory`
  /// (the layout of spot_wrapper/testing/credentials).
  static TlsConfig FromDirectory(const std::string& directory);
};

struct ServerConfig {
  std::string address = "0.0.0.0:443";
  TlsConfig tls;
  /// Optional file listing unimplemented methods clients called.
  std::string unimplemented_report_file;
};

/// Records API methods the simulator does not implement; logs each to
/// stderr the first time a client calls it and, if `report_file` is set,
/// appends it there (one method per line; the file is truncated first).
/// Thread-safe.
class UnimplementedCalls {
 public:
  explicit UnimplementedCalls(std::string report_file = "");
  void Report(const std::string& method);
  std::set<std::string> Methods() const;

 private:
  std::string report_file_;
  mutable std::mutex mutex_;
  std::set<std::string> methods_;
};

/// gRPC server exposing the Spot API of a simulated Robot.
class SpotApiServer {
 public:
  SpotApiServer(Robot& robot, const ServerConfig& config);
  ~SpotApiServer();

  SpotApiServer(const SpotApiServer&) = delete;
  SpotApiServer& operator=(const SpotApiServer&) = delete;

  /// Port actually bound (useful when the address requests port 0).
  int port() const { return port_; }

  /// Methods clients called that answered UNIMPLEMENTED, e.g.
  /// "/bosdyn.api.docking.DockingService/GetDockingState".
  std::set<std::string> UnimplementedMethods() const { return unimplemented_->Methods(); }

  void Shutdown();

 private:
  std::shared_ptr<UnimplementedCalls> unimplemented_;
  std::vector<std::unique_ptr<grpc::Service>> services_;
  std::unique_ptr<grpc::CallbackGenericService> unknown_services_;
  std::unique_ptr<grpc::Server> server_;
  int port_ = 0;
};

}  // namespace spot_sim
