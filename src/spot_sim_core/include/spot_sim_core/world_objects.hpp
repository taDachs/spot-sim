#pragma once

#include <map>
#include <mutex>

#include <bosdyn/api/world_object.pb.h>

#include "spot_sim_core/time.hpp"

namespace spot_sim {

/// WorldObjectService semantics for objects added by clients (the simulated
/// robot perceives nothing itself): add / change / delete by id, listing
/// with type and timestamp filters, expiry after object_lifetime. Thread-safe.
class WorldObjects {
 public:
  bosdyn::api::MutateWorldObjectResponse Mutate(const bosdyn::api::MutateWorldObjectRequest& request, RobotTime now);
  bosdyn::api::ListWorldObjectResponse List(const bosdyn::api::ListWorldObjectRequest& request, RobotTime now);

 private:
  void Expire(RobotTime now);

  std::mutex mutex_;
  std::map<int32_t, bosdyn::api::WorldObject> objects_;
  int32_t next_id_ = 1;
};

/// The WorldObjectType an object counts as, from the properties it carries.
bosdyn::api::WorldObjectType TypeOf(const bosdyn::api::WorldObject& object);

}  // namespace spot_sim
