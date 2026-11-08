#include "spot_sim_core/estop.hpp"

#include <cstdio>

namespace spot_sim {

namespace api = bosdyn::api;

EstopSystem::EstopSystem() : config_id_("spot-sim-estop-config-0"), next_config_id_(1), rng_(std::random_device{}()) {}

api::EstopConfig EstopSystem::ActiveConfig() const {
  api::EstopConfig config;
  config.set_unique_id(config_id_);
  for (const auto& slot : slots_) {
    *config.add_endpoints() = slot.endpoint;
  }
  return config;
}

api::GetEstopConfigResponse EstopSystem::GetConfig(const api::GetEstopConfigRequest& request) const {
  api::GetEstopConfigResponse response;
  *response.mutable_request() = request;
  *response.mutable_active_config() = ActiveConfig();
  return response;
}

api::SetEstopConfigResponse EstopSystem::SetConfig(const api::SetEstopConfigRequest& request, bool motors_on) {
  api::SetEstopConfigResponse response;
  *response.mutable_request() = request;
  if (request.target_config_id() != config_id_) {
    response.set_status(api::SetEstopConfigResponse::STATUS_INVALID_ID);
  } else if (motors_on) {
    response.set_status(api::SetEstopConfigResponse::STATUS_MOTORS_ON);
  } else {
    config_id_ = "spot-sim-estop-config-" + std::to_string(next_config_id_++);
    slots_.clear();
    for (const auto& endpoint : request.config().endpoints()) {
      Slot slot;
      slot.endpoint = endpoint;
      if (slot.endpoint.unique_id().empty()) {
        slot.endpoint.set_unique_id("spot-sim-estop-endpoint-" + std::to_string(next_endpoint_id_++));
      }
      slots_.push_back(slot);
    }
    response.set_status(api::SetEstopConfigResponse::STATUS_SUCCESS);
  }
  *response.mutable_active_config() = ActiveConfig();
  return response;
}

EstopSystem::Slot* EstopSystem::FindSlot(const api::EstopEndpoint& target, const api::EstopEndpoint& fallback) {
  // Prefer an explicit unique id; the Python SDK only sets the role on the
  // target and carries the unique id in the new endpoint.
  for (const std::string& id : {target.unique_id(), fallback.unique_id()}) {
    if (id.empty()) continue;
    for (auto& slot : slots_) {
      if (slot.endpoint.unique_id() == id) return &slot;
    }
    return nullptr;
  }
  for (auto& slot : slots_) {
    if (slot.endpoint.role() == target.role()) return &slot;
  }
  return nullptr;
}

api::RegisterEstopEndpointResponse EstopSystem::Register(const api::RegisterEstopEndpointRequest& request) {
  api::RegisterEstopEndpointResponse response;
  *response.mutable_request() = request;
  if (request.target_config_id() != config_id_) {
    response.set_status(api::RegisterEstopEndpointResponse::STATUS_CONFIG_MISMATCH);
    return response;
  }
  if (request.new_endpoint().timeout().seconds() <= 0 && request.new_endpoint().timeout().nanos() <= 0) {
    response.set_status(api::RegisterEstopEndpointResponse::STATUS_INVALID_ENDPOINT);
    return response;
  }
  Slot* slot = FindSlot(request.target_endpoint(), request.new_endpoint());
  if (slot == nullptr) {
    response.set_status(api::RegisterEstopEndpointResponse::STATUS_ENDPOINT_MISMATCH);
    return response;
  }
  const std::string unique_id = slot->endpoint.unique_id();
  slot->endpoint = request.new_endpoint();
  slot->endpoint.set_unique_id(unique_id);
  slot->registered = true;
  slot->checked_in = false;
  slot->challenge_issued = false;
  slot->level = api::ESTOP_LEVEL_CUT;
  *response.mutable_new_endpoint() = slot->endpoint;
  response.set_status(api::RegisterEstopEndpointResponse::STATUS_SUCCESS);
  std::fprintf(stderr, "[estop] registered endpoint '%s' (%s) in %s\n", slot->endpoint.name().c_str(),
               unique_id.c_str(), config_id_.c_str());
  return response;
}

api::DeregisterEstopEndpointResponse EstopSystem::Deregister(const api::DeregisterEstopEndpointRequest& request,
                                                             bool motors_on) {
  api::DeregisterEstopEndpointResponse response;
  *response.mutable_request() = request;
  if (request.target_config_id() != config_id_) {
    response.set_status(api::DeregisterEstopEndpointResponse::STATUS_CONFIG_MISMATCH);
    return response;
  }
  Slot* slot = FindSlot(request.target_endpoint(), request.target_endpoint());
  if (slot == nullptr || !slot->registered) {
    response.set_status(api::DeregisterEstopEndpointResponse::STATUS_ENDPOINT_MISMATCH);
    return response;
  }
  if (motors_on) {
    response.set_status(api::DeregisterEstopEndpointResponse::STATUS_MOTORS_ON);
    return response;
  }
  // The slot stays in the configuration, so the robot stays stopped until
  // someone registers for it again.
  slot->registered = false;
  slot->checked_in = false;
  response.set_status(api::DeregisterEstopEndpointResponse::STATUS_SUCCESS);
  return response;
}

api::EstopCheckInResponse EstopSystem::CheckIn(const api::EstopCheckInRequest& request, WallTime now) {
  api::EstopCheckInResponse response;
  *response.mutable_request() = request;

  Slot* slot = nullptr;
  for (auto& candidate : slots_) {
    if (candidate.registered && candidate.endpoint.unique_id() == request.endpoint().unique_id()) {
      slot = &candidate;
    }
  }
  if (slot == nullptr) {
    response.set_status(api::EstopCheckInResponse::STATUS_ENDPOINT_UNKNOWN);
    return response;
  }

  // The SDK's keepalive thread and an explicit stop/allow can check in
  // concurrently with the same challenge; accept the previous one as well so
  // the second check-in (e.g. a hard stop) is not lost.
  const bool response_ok = !slot->challenge_issued || request.response() == ~slot->challenge ||
                           request.response() == ~slot->previous_challenge;
  slot->previous_challenge = slot->challenge;
  slot->challenge = rng_();
  slot->challenge_issued = true;
  response.set_challenge(slot->challenge);
  if (!response_ok) {
    response.set_status(api::EstopCheckInResponse::STATUS_INCORRECT_CHALLENGE_RESPONSE);
    return response;
  }

  slot->checked_in = true;
  slot->last_valid_check_in = now;
  const auto new_level =
      request.stop_level() == api::ESTOP_LEVEL_UNKNOWN ? api::ESTOP_LEVEL_CUT : request.stop_level();
  if (new_level != slot->level) {
    std::fprintf(stderr, "[estop] endpoint '%s' (%s): %s -> %s\n", slot->endpoint.name().c_str(),
                 slot->endpoint.unique_id().c_str(), api::EstopStopLevel_Name(slot->level).c_str(),
                 api::EstopStopLevel_Name(new_level).c_str());
  }
  slot->level = new_level;
  response.set_status(api::EstopCheckInResponse::STATUS_OK);
  return response;
}

api::EstopStopLevel EstopSystem::LevelOf(const Slot& slot, WallTime now) const {
  if (!slot.registered || !slot.checked_in) {
    return api::ESTOP_LEVEL_CUT;
  }
  const auto timeout = FromDuration(slot.endpoint.timeout());
  if (now - slot.last_valid_check_in > timeout) {
    return api::ESTOP_LEVEL_CUT;
  }
  return slot.level;
}

api::EstopStopLevel EstopSystem::StopLevel(WallTime now) const {
  api::EstopStopLevel level = api::ESTOP_LEVEL_NONE;
  for (const auto& slot : slots_) {
    const auto slot_level = LevelOf(slot, now);
    if (MoreSevere(slot_level, level)) {
      level = slot_level;
    }
  }
  return level;
}

api::GetEstopSystemStatusResponse EstopSystem::Status(WallTime now) const {
  api::GetEstopSystemStatusResponse response;
  auto* status = response.mutable_status();
  for (const auto& slot : slots_) {
    auto* entry = status->add_endpoints();
    *entry->mutable_endpoint() = slot.endpoint;
    entry->set_stop_level(LevelOf(slot, now));
    if (slot.checked_in) {
      *entry->mutable_time_since_valid_response() = ToDuration(now - slot.last_valid_check_in);
    }
  }
  status->set_stop_level(StopLevel(now));
  return response;
}

std::vector<EstopSample> EstopSystem::States(WallTime now) const {
  std::vector<EstopSample> states;
  for (const auto& slot : slots_) {
    states.push_back(EstopSample{slot.endpoint.name().empty() ? slot.endpoint.role() : slot.endpoint.name(),
                                 bosdyn::api::EStopState::TYPE_SOFTWARE,
                                 LevelOf(slot, now) != api::ESTOP_LEVEL_NONE});
  }
  return states;
}

}  // namespace spot_sim
