#include "spot_sim_core/command_request.hpp"

namespace spot_sim {

namespace api = bosdyn::api;

std::variant<RobotTime, Rejection> CheckEndTime(const std::string& command, bool has_end_time,
                                                const google::protobuf::Timestamp& end_time, bool clock_synced,
                                                RobotTime now) {
  if (!has_end_time) {
    return Rejection{api::RobotCommandResponse::STATUS_INVALID_REQUEST, command + " requires an end_time"};
  }
  if (!clock_synced) {
    return Rejection{api::RobotCommandResponse::STATUS_NO_TIMESYNC, "clock_identifier has no established time sync"};
  }
  const RobotTime end = FromTimestamp(end_time);
  if (end <= now) {
    return Rejection{api::RobotCommandResponse::STATUS_EXPIRED, "end_time has already passed"};
  }
  return end;
}

}  // namespace spot_sim
