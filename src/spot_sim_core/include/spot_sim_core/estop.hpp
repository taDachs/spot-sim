#pragma once

#include <cstdint>
#include <random>
#include <string>
#include <vector>

#include <bosdyn/api/estop.pb.h>

#include "spot_sim_core/robot_state.hpp"
#include "spot_sim_core/time.hpp"

namespace spot_sim {

/// Software E-Stop system, as the robot's EstopService implements it.
///
/// A configuration lists endpoint slots. Every slot must be registered and
/// checking in (challenge/response) within its timeout, otherwise it counts
/// as ESTOP_LEVEL_CUT. The system stop level is the most severe level of all
/// slots; with no slots configured the robot is not stopped.
///
/// Deviation from the real robot: an endpoint's first check-in is accepted
/// without a valid challenge response (the client cannot have one yet), so
/// the robot can be powered on right after claiming.
///
/// Check-in deadlines use wall-clock time. Not thread-safe; Robot serializes.
class EstopSystem {
 public:
  EstopSystem();

  bosdyn::api::GetEstopConfigResponse GetConfig(const bosdyn::api::GetEstopConfigRequest& request) const;
  bosdyn::api::SetEstopConfigResponse SetConfig(const bosdyn::api::SetEstopConfigRequest& request, bool motors_on);
  bosdyn::api::RegisterEstopEndpointResponse Register(const bosdyn::api::RegisterEstopEndpointRequest& request);
  bosdyn::api::DeregisterEstopEndpointResponse Deregister(const bosdyn::api::DeregisterEstopEndpointRequest& request,
                                                          bool motors_on);
  bosdyn::api::EstopCheckInResponse CheckIn(const bosdyn::api::EstopCheckInRequest& request, WallTime now);
  bosdyn::api::GetEstopSystemStatusResponse Status(WallTime now) const;

  bosdyn::api::EstopStopLevel StopLevel(WallTime now) const;

  /// Software E-Stop entries for RobotState.estop_states.
  std::vector<EstopSample> States(WallTime now) const;

 private:
  struct Slot {
    bosdyn::api::EstopEndpoint endpoint;
    bool registered = false;
    bool checked_in = false;
    bosdyn::api::EstopStopLevel level = bosdyn::api::ESTOP_LEVEL_CUT;
    WallTime last_valid_check_in{};
    bool challenge_issued = false;
    uint64_t challenge = 0;
    uint64_t previous_challenge = 0;  ///< accepted too: periodic and explicit check-ins race
  };

  bosdyn::api::EstopStopLevel LevelOf(const Slot& slot, WallTime now) const;
  bosdyn::api::EstopConfig ActiveConfig() const;
  Slot* FindSlot(const bosdyn::api::EstopEndpoint& target, const bosdyn::api::EstopEndpoint& fallback);

  std::string config_id_;
  int next_config_id_ = 0;
  int next_endpoint_id_ = 0;
  std::vector<Slot> slots_;
  std::mt19937_64 rng_;
};

/// Severity order: CUT (most severe) < SETTLE_THEN_CUT < NONE.
inline bool MoreSevere(bosdyn::api::EstopStopLevel a, bosdyn::api::EstopStopLevel b) {
  auto rank = [](bosdyn::api::EstopStopLevel level) {
    switch (level) {
      case bosdyn::api::ESTOP_LEVEL_NONE:
        return 2;
      case bosdyn::api::ESTOP_LEVEL_SETTLE_THEN_CUT:
        return 1;
      default:
        return 0;  // CUT and UNKNOWN
    }
  };
  return rank(a) < rank(b);
}

}  // namespace spot_sim
