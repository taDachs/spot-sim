#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

#include <bosdyn/api/lease.pb.h>

#include "spot_sim_core/time.hpp"

namespace spot_sim {

/// Spot lease bookkeeping for the resource tree all-leases > body > mobility.
///
/// One lease covers "body" and its sub-resource "mobility": acquiring either
/// claims both, and a body lease is valid for mobility commands. Each
/// acquire/take starts a new sequence number, which makes every earlier lease
/// "older". A lease nobody has used or retained for `stale_timeout` (wall
/// time) can be acquired without taking it.
///
/// Not thread-safe; Robot serializes access.
class LeaseManager {
 public:
  explicit LeaseManager(std::chrono::nanoseconds stale_timeout = std::chrono::seconds(10));

  bosdyn::api::AcquireLeaseResponse Acquire(const bosdyn::api::AcquireLeaseRequest& request, WallTime now);
  bosdyn::api::TakeLeaseResponse Take(const bosdyn::api::TakeLeaseRequest& request, WallTime now);
  bosdyn::api::ReturnLeaseResponse Return(const bosdyn::api::ReturnLeaseRequest& request, WallTime now);
  bosdyn::api::RetainLeaseResponse Retain(const bosdyn::api::RetainLeaseRequest& request, WallTime now);
  bosdyn::api::ListLeasesResponse List(WallTime now) const;

  /// Validates a lease attached to a command and, if valid, counts as use
  /// (keeps it fresh).
  bosdyn::api::LeaseUseResult Use(const bosdyn::api::Lease& attempted, WallTime now);

  const std::string& epoch() const { return epoch_; }

 private:
  static bool IsKnownResource(const std::string& resource);
  bool IsStale(WallTime now) const;
  bosdyn::api::Lease Grant(const std::string& resource, const std::string& client_name, WallTime now);
  bosdyn::api::Lease LeaseFor(const std::string& resource) const;

  std::chrono::nanoseconds stale_timeout_;
  std::string epoch_;
  uint32_t sequence_ = 0;
  bool owned_ = false;
  bosdyn::api::LeaseOwner owner_;
  std::string client_name_;
  WallTime last_used_{};
};

}  // namespace spot_sim
