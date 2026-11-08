#include "spot_sim_core/lease.hpp"

#include <iomanip>
#include <random>
#include <sstream>
#include <vector>

namespace spot_sim {

namespace api = bosdyn::api;

namespace {
constexpr const char* kRoot = "all-leases";
constexpr const char* kBody = "body";
constexpr const char* kMobility = "mobility";
constexpr const char* kFullArm = "full-arm";
constexpr const char* kArm = "arm";
constexpr const char* kGripper = "gripper";

/// A fresh epoch per simulator run, so leases from a previous run are rejected.
std::string NewEpoch() {
  std::random_device rd;
  std::ostringstream out;
  out << "spot-sim-" << std::hex << std::setw(8) << std::setfill('0') << rd();
  return out.str();
}
}  // namespace

LeaseManager::LeaseManager(std::chrono::nanoseconds stale_timeout) : stale_timeout_(stale_timeout), epoch_(NewEpoch()) {}

bool LeaseManager::IsKnownResource(const std::string& resource) const {
  if (resource == kBody || resource == kMobility) {
    return true;
  }
  return has_arm_ && (resource == kFullArm || resource == kArm || resource == kGripper);
}

bool LeaseManager::IsStale(WallTime now) const { return !owned_ || now - last_used_ > stale_timeout_; }

api::Lease LeaseManager::LeaseFor(const std::string& resource) const {
  api::Lease lease;
  lease.set_resource(resource);
  lease.set_epoch(epoch_);
  lease.add_sequence(sequence_);
  if (!client_name_.empty()) {
    lease.add_client_names(client_name_);
  }
  return lease;
}

api::Lease LeaseManager::Grant(const std::string& resource, const std::string& client_name, WallTime now) {
  ++sequence_;
  owned_ = true;
  client_name_ = client_name;
  owner_.set_client_name(client_name);
  last_used_ = now;
  return LeaseFor(resource);
}

api::AcquireLeaseResponse LeaseManager::Acquire(const api::AcquireLeaseRequest& request, WallTime now) {
  api::AcquireLeaseResponse response;
  if (!IsKnownResource(request.resource())) {
    response.set_status(api::AcquireLeaseResponse::STATUS_INVALID_RESOURCE);
    return response;
  }
  if (!IsStale(now)) {
    response.set_status(api::AcquireLeaseResponse::STATUS_RESOURCE_ALREADY_CLAIMED);
    *response.mutable_lease_owner() = owner_;
    return response;
  }
  *response.mutable_lease() = Grant(request.resource(), request.header().client_name(), now);
  *response.mutable_lease_owner() = owner_;
  response.set_status(api::AcquireLeaseResponse::STATUS_OK);
  return response;
}

api::TakeLeaseResponse LeaseManager::Take(const api::TakeLeaseRequest& request, WallTime now) {
  api::TakeLeaseResponse response;
  if (!IsKnownResource(request.resource())) {
    response.set_status(api::TakeLeaseResponse::STATUS_INVALID_RESOURCE);
    return response;
  }
  *response.mutable_lease() = Grant(request.resource(), request.header().client_name(), now);
  *response.mutable_lease_owner() = owner_;
  response.set_status(api::TakeLeaseResponse::STATUS_OK);
  return response;
}

api::ReturnLeaseResponse LeaseManager::Return(const api::ReturnLeaseRequest& request, WallTime now) {
  api::ReturnLeaseResponse response;
  if (!IsKnownResource(request.lease().resource())) {
    response.set_status(api::ReturnLeaseResponse::STATUS_INVALID_RESOURCE);
    return response;
  }
  if (Use(request.lease(), now).status() != api::LeaseUseResult::STATUS_OK) {
    response.set_status(api::ReturnLeaseResponse::STATUS_NOT_ACTIVE_LEASE);
    return response;
  }
  owned_ = false;
  owner_.Clear();
  client_name_.clear();
  response.set_status(api::ReturnLeaseResponse::STATUS_OK);
  return response;
}

api::RetainLeaseResponse LeaseManager::Retain(const api::RetainLeaseRequest& request, WallTime now) {
  api::RetainLeaseResponse response;
  *response.mutable_lease_use_result() = Use(request.lease(), now);
  return response;
}

api::LeaseUseResult LeaseManager::Use(const api::Lease& attempted, WallTime now) {
  api::LeaseUseResult result;
  *result.mutable_attempted_lease() = attempted;
  if (owned_) {
    *result.mutable_latest_known_lease() = LeaseFor(attempted.resource().empty() ? kBody : attempted.resource());
    *result.mutable_owner() = owner_;
  }

  if (!IsKnownResource(attempted.resource()) || attempted.sequence_size() == 0) {
    result.set_status(api::LeaseUseResult::STATUS_INVALID_LEASE);
  } else if (attempted.epoch() != epoch_) {
    result.set_status(api::LeaseUseResult::STATUS_WRONG_EPOCH);
  } else if (attempted.sequence(0) < sequence_) {
    result.set_status(api::LeaseUseResult::STATUS_OLDER);
  } else if (attempted.sequence(0) > sequence_) {
    result.set_status(api::LeaseUseResult::STATUS_INVALID_LEASE);
  } else if (!owned_) {
    // Current sequence, but it was returned.
    result.set_status(api::LeaseUseResult::STATUS_REVOKED);
  } else {
    // Current lease or a sublease of it.
    result.set_status(api::LeaseUseResult::STATUS_OK);
    last_used_ = now;
  }
  return result;
}

api::ListLeasesResponse LeaseManager::List(WallTime now) const {
  api::ListLeasesResponse response;
  auto* tree = response.mutable_resource_tree();
  tree->set_resource(kRoot);
  auto* body = tree->add_sub_resources();
  body->set_resource(kBody);
  body->add_sub_resources()->set_resource(kMobility);
  std::vector<const char*> names = {kBody, kMobility};
  if (has_arm_) {
    auto* full_arm = body->add_sub_resources();
    full_arm->set_resource(kFullArm);
    full_arm->add_sub_resources()->set_resource(kArm);
    full_arm->add_sub_resources()->set_resource(kGripper);
    names.insert(names.end(), {kFullArm, kArm, kGripper});
  }

  for (const char* name : names) {
    auto* resource = response.add_resources();
    resource->set_resource(name);
    if (sequence_ > 0) {
      *resource->mutable_lease() = LeaseFor(name);
    }
    if (owned_) {
      *resource->mutable_lease_owner() = owner_;
    }
    resource->set_is_stale(IsStale(now));
  }
  return response;
}

}  // namespace spot_sim
