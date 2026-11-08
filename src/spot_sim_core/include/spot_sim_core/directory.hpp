#pragma once

#include <optional>
#include <string>
#include <vector>

#include <bosdyn/api/directory.pb.h>

namespace spot_sim {

/// Server name (TLS authority) for most services.
inline constexpr const char* kApiAuthority = "api.spot.robot";

/// Services advertised by the simulated robot's directory.
///
/// Every service that SpotWrapper or the C++ driver nodes create a client for
/// at startup must be listed, even if it is not implemented yet: a missing
/// entry makes SpotWrapper retry its initialization forever. Unimplemented
/// RPCs answer UNIMPLEMENTED.
///
/// `gripper-camera-param` is deliberately absent: the C++ driver nodes infer
/// "has arm" from its presence.
std::vector<bosdyn::api::ServiceEntry> DefaultServiceEntries();

std::optional<bosdyn::api::ServiceEntry> FindServiceEntry(const std::vector<bosdyn::api::ServiceEntry>& entries,
                                                          const std::string& name);

}  // namespace spot_sim
