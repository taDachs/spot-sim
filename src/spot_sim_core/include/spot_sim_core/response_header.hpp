#pragma once

#include <bosdyn/api/header.pb.h>

#include "spot_sim_core/time.hpp"

namespace spot_sim {

/// Fills the standard ResponseHeader the Spot SDK expects on every response:
/// echo of the request header, robot-clock receive/response timestamps and
/// an error code (CODE_OK unless the caller already set one).
template <typename Request, typename Response>
void FillResponseHeader(const Request& request, Response* response, RobotTime received, RobotTime now) {
  auto* header = response->mutable_header();
  *header->mutable_request_header() = request.header();
  *header->mutable_request_received_timestamp() = ToTimestamp(received);
  *header->mutable_response_timestamp() = ToTimestamp(now);
  if (header->error().code() == bosdyn::api::CommonError::CODE_UNSPECIFIED) {
    header->mutable_error()->set_code(bosdyn::api::CommonError::CODE_OK);
  }
}

}  // namespace spot_sim
