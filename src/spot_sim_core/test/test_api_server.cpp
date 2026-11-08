#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>

#include <grpcpp/grpcpp.h>

#include <bosdyn/api/auth_service.grpc.pb.h>
#include <bosdyn/api/directory_service.grpc.pb.h>
#include <bosdyn/api/robot_command_service.grpc.pb.h>
#include <bosdyn/api/robot_state_service.grpc.pb.h>
#include <bosdyn/api/time_sync_service.grpc.pb.h>

#include "spot_sim_core/api_server.hpp"
#include "spot_sim_core/robot.hpp"

using namespace std::chrono_literals;

namespace {

std::string CertDir() {
  const char* dir = std::getenv("SPOT_SIM_TEST_CERTS_DIR");
  return dir ? dir : "";
}

std::string ReadFile(const std::filesystem::path& path) {
  std::ifstream in(path);
  std::stringstream buffer;
  buffer << in.rdbuf();
  return buffer.str();
}

class ApiServerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    if (CertDir().empty()) {
      GTEST_SKIP() << "SPOT_SIM_TEST_CERTS_DIR not set";
    }
    robot_ = std::make_unique<spot_sim::Robot>(spot_sim::RobotConfig{});
    robot_->Step(42s);
    spot_sim::ServerConfig config;
    config.address = "127.0.0.1:0";
    config.tls = spot_sim::TlsConfig::FromDirectory(CertDir());
    server_ = std::make_unique<spot_sim::SpotApiServer>(*robot_, config);
  }

  /// Channel that verifies the server against the test CA under `authority`,
  /// the way the Spot SDK does.
  std::shared_ptr<grpc::Channel> Channel(const std::string& authority) {
    grpc::SslCredentialsOptions ssl;
    ssl.pem_root_certs = ReadFile(std::filesystem::path(CertDir()) / "ca.crt");
    grpc::ChannelArguments args;
    args.SetSslTargetNameOverride(authority);
    return grpc::CreateCustomChannel("127.0.0.1:" + std::to_string(server_->port()), grpc::SslCredentials(ssl), args);
  }

  std::unique_ptr<spot_sim::Robot> robot_;
  std::unique_ptr<spot_sim::SpotApiServer> server_;
};

}  // namespace

TEST_F(ApiServerTest, AuthenticatesOverTlsWithAuthAuthority) {
  auto stub = bosdyn::api::AuthService::NewStub(Channel("auth.spot.robot"));
  grpc::ClientContext context;
  bosdyn::api::GetAuthTokenRequest request;
  request.set_username("user");
  request.set_password("password");
  bosdyn::api::GetAuthTokenResponse response;
  const auto status = stub->GetAuthToken(&context, request, &response);
  ASSERT_TRUE(status.ok()) << status.error_message();
  EXPECT_EQ(response.status(), bosdyn::api::GetAuthTokenResponse::STATUS_OK);
  EXPECT_FALSE(response.token().empty());
  EXPECT_EQ(response.header().error().code(), bosdyn::api::CommonError::CODE_OK);
  EXPECT_EQ(response.header().response_timestamp().seconds(), 42);
}

TEST_F(ApiServerTest, RejectsWrongPassword) {
  auto stub = bosdyn::api::AuthService::NewStub(Channel("auth.spot.robot"));
  grpc::ClientContext context;
  bosdyn::api::GetAuthTokenRequest request;
  request.set_username("user");
  request.set_password("wrong");
  bosdyn::api::GetAuthTokenResponse response;
  ASSERT_TRUE(stub->GetAuthToken(&context, request, &response).ok());
  EXPECT_EQ(response.status(), bosdyn::api::GetAuthTokenResponse::STATUS_INVALID_LOGIN);
}

TEST_F(ApiServerTest, ServesApiAuthorityAndEchoesRequestHeader) {
  auto stub = bosdyn::api::DirectoryService::NewStub(Channel("api.spot.robot"));
  grpc::ClientContext context;
  bosdyn::api::ListServiceEntriesRequest request;
  request.mutable_header()->set_client_name("test-client");
  bosdyn::api::ListServiceEntriesResponse response;
  ASSERT_TRUE(stub->ListServiceEntries(&context, request, &response).ok());
  EXPECT_GT(response.service_entries_size(), 10);
  EXPECT_EQ(response.header().request_header().client_name(), "test-client");
}

TEST_F(ApiServerTest, TimeSyncUsesRobotClock) {
  auto stub = bosdyn::api::TimeSyncService::NewStub(Channel("api.spot.robot"));
  grpc::ClientContext context;
  bosdyn::api::TimeSyncUpdateRequest request;
  bosdyn::api::TimeSyncUpdateResponse response;
  ASSERT_TRUE(stub->TimeSyncUpdate(&context, request, &response).ok());
  EXPECT_FALSE(response.clock_identifier().empty());
  EXPECT_EQ(response.header().request_received_timestamp().seconds(), 42);
}

TEST_F(ApiServerTest, UnimplementedRpcsFailFast) {
  // RobotCommandService is listed in the directory but not served yet: the
  // call must fail immediately instead of hanging.
  auto stub = bosdyn::api::RobotCommandService::NewStub(Channel("api.spot.robot"));
  grpc::ClientContext context;
  context.set_deadline(std::chrono::system_clock::now() + 5s);
  bosdyn::api::RobotCommandRequest request;
  bosdyn::api::RobotCommandResponse response;
  const auto status = stub->RobotCommand(&context, request, &response);
  EXPECT_EQ(status.error_code(), grpc::StatusCode::UNIMPLEMENTED);
}

TEST_F(ApiServerTest, ServesRobotState) {
  auto stub = bosdyn::api::RobotStateService::NewStub(Channel("api.spot.robot"));
  grpc::ClientContext context;
  bosdyn::api::RobotStateRequest request;
  bosdyn::api::RobotStateResponse response;
  ASSERT_TRUE(stub->GetRobotState(&context, request, &response).ok());
  EXPECT_EQ(response.robot_state().kinematic_state().joint_states_size(), 12);
}
