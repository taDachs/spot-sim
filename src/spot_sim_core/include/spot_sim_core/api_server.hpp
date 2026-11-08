#pragma once

#include <memory>
#include <string>
#include <vector>

namespace grpc {
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

  void Shutdown();

 private:
  std::vector<std::unique_ptr<grpc::Service>> services_;
  std::unique_ptr<grpc::Server> server_;
  int port_ = 0;
};

}  // namespace spot_sim
