#include "spot_sim_core/directory.hpp"

namespace spot_sim {

namespace {
bosdyn::api::ServiceEntry Entry(const std::string& name, const std::string& type,
                                const std::string& authority = kApiAuthority, bool user_token_required = true) {
  bosdyn::api::ServiceEntry entry;
  entry.set_name(name);
  entry.set_type(type);
  entry.set_authority(authority);
  entry.set_user_token_required(user_token_required);
  return entry;
}
}  // namespace

std::vector<bosdyn::api::ServiceEntry> DefaultServiceEntries(bool has_arm) {
  std::vector<bosdyn::api::ServiceEntry> entries = {
      // Bootstrap services with their own authorities.
      Entry("auth", "bosdyn.api.AuthService", "auth.spot.robot", false),
      Entry("directory", "bosdyn.api.DirectoryService"),
      Entry("robot-id", "bosdyn.api.RobotIdService", "id.spot.robot", false),
      // Core services.
      Entry("time-sync", "bosdyn.api.TimeSyncService"),
      Entry("license", "bosdyn.api.LicenseService"),
      Entry("lease", "bosdyn.api.LeaseService"),
      Entry("estop", "bosdyn.api.EstopService"),
      Entry("power", "bosdyn.api.PowerService"),
      Entry("robot-state", "bosdyn.api.RobotStateService"),
      Entry("robot-command", "bosdyn.api.RobotCommandService"),
      Entry("image", "bosdyn.api.ImageService"),
      Entry("world-objects", "bosdyn.api.WorldObjectService"),
      Entry("inverse-kinematics", "bosdyn.api.spot.InverseKinematicsService"),
      // Listed so clients can be created at startup; RPCs are not implemented yet.
      Entry("graph-nav-service", "bosdyn.api.graph_nav.GraphNavService"),
      Entry("map-processing-service", "bosdyn.api.graph_nav.MapProcessingService"),
      Entry("docking", "bosdyn.api.docking.DockingService"),
      Entry("spot-check", "bosdyn.api.spot.SpotCheckService"),
      Entry("robot-mission", "bosdyn.api.mission.MissionService"),
  };
  if (has_arm) {
    entries.push_back(Entry("manipulation", "bosdyn.api.ManipulationApiService"));
    entries.push_back(Entry("arm-surface-contact", "bosdyn.api.ArmSurfaceContactService"));
    entries.push_back(Entry("gripper-camera-param", "bosdyn.api.GripperCameraParamService"));
  }
  return entries;
}

std::optional<bosdyn::api::ServiceEntry> FindServiceEntry(const std::vector<bosdyn::api::ServiceEntry>& entries,
                                                          const std::string& name) {
  for (const auto& entry : entries) {
    if (entry.name() == name) {
      return entry;
    }
  }
  return std::nullopt;
}

}  // namespace spot_sim
