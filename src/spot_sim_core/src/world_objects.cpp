#include "spot_sim_core/world_objects.hpp"

#include <algorithm>

namespace spot_sim {

namespace api = bosdyn::api;

api::WorldObjectType TypeOf(const api::WorldObject& object) {
  if (object.has_apriltag_properties()) return api::WORLD_OBJECT_APRILTAG;
  if (object.has_dock_properties()) return api::WORLD_OBJECT_DOCK;
  if (object.has_image_properties()) return api::WORLD_OBJECT_IMAGE_COORDINATES;
  if (object.has_nogo_region_properties()) return api::WORLD_OBJECT_USER_NOGO;
  if (object.has_staircase_properties()) return api::WORLD_OBJECT_STAIRCASE;
  if (object.drawable_properties_size() > 0) return api::WORLD_OBJECT_DRAWABLE;
  return api::WORLD_OBJECT_UNKNOWN;
}

void WorldObjects::Expire(RobotTime now) {
  for (auto it = objects_.begin(); it != objects_.end();) {
    const auto& object = it->second;
    const bool expires = object.has_object_lifetime() && object.has_acquisition_time() &&
                         FromDuration(object.object_lifetime()) > std::chrono::nanoseconds{0};
    if (expires && FromTimestamp(object.acquisition_time()) + FromDuration(object.object_lifetime()) < now) {
      it = objects_.erase(it);
    } else {
      ++it;
    }
  }
}

api::MutateWorldObjectResponse WorldObjects::Mutate(const api::MutateWorldObjectRequest& request, RobotTime now) {
  std::lock_guard<std::mutex> lock(mutex_);
  Expire(now);
  api::MutateWorldObjectResponse response;
  const auto& mutation = request.mutation();
  api::WorldObject object = mutation.object();
  if (!object.has_acquisition_time()) {
    *object.mutable_acquisition_time() = ToTimestamp(now);
  }
  switch (mutation.action()) {
    case api::MutateWorldObjectRequest::ACTION_ADD: {
      const int32_t id = next_id_++;
      object.set_id(id);
      objects_[id] = object;
      response.set_mutated_object_id(id);
      response.set_status(api::MutateWorldObjectResponse::STATUS_OK);
      break;
    }
    case api::MutateWorldObjectRequest::ACTION_CHANGE:
    case api::MutateWorldObjectRequest::ACTION_DELETE: {
      const auto it = objects_.find(object.id());
      if (it == objects_.end()) {
        response.set_status(api::MutateWorldObjectResponse::STATUS_INVALID_MUTATION_ID);
        break;
      }
      if (mutation.action() == api::MutateWorldObjectRequest::ACTION_CHANGE) {
        it->second = object;
      } else {
        objects_.erase(it);
      }
      response.set_mutated_object_id(object.id());
      response.set_status(api::MutateWorldObjectResponse::STATUS_OK);
      break;
    }
    default:
      response.set_status(api::MutateWorldObjectResponse::STATUS_INVALID_WORLD_OBJECT);
      break;
  }
  return response;
}

api::ListWorldObjectResponse WorldObjects::List(const api::ListWorldObjectRequest& request, RobotTime now) {
  std::lock_guard<std::mutex> lock(mutex_);
  Expire(now);
  api::ListWorldObjectResponse response;
  const bool filter_time = request.has_timestamp_filter();
  const RobotTime after = filter_time ? FromTimestamp(request.timestamp_filter()) : RobotTime{0};
  for (const auto& [id, object] : objects_) {
    const auto& types = request.object_type();
    if (!types.empty() && std::find(types.begin(), types.end(), TypeOf(object)) == types.end()) {
      continue;
    }
    if (filter_time && FromTimestamp(object.acquisition_time()) <= after) {
      continue;
    }
    *response.add_world_objects() = object;
  }
  return response;
}

}  // namespace spot_sim
