#include <gtest/gtest.h>

#include "spot_sim_core/directory.hpp"

TEST(Directory, ListsEveryServiceTheDriverCreatesAClientFor) {
  const auto entries = spot_sim::DefaultServiceEntries();
  // Clients created by SpotWrapper (spot_wrapper/wrapper.py) and the C++
  // driver nodes (spot_driver/src/api/default_spot_api.cpp) at startup.
  for (const char* name : {"auth", "directory", "robot-id", "time-sync", "license", "lease", "estop", "power",
                           "robot-state", "robot-command", "image", "world-objects", "graph-nav-service",
                           "map-processing-service", "docking", "spot-check", "robot-mission"}) {
    EXPECT_TRUE(spot_sim::FindServiceEntry(entries, name).has_value()) << name;
  }
}

TEST(Directory, UsesBootstrapAuthorities) {
  const auto entries = spot_sim::DefaultServiceEntries();
  EXPECT_EQ(spot_sim::FindServiceEntry(entries, "auth")->authority(), "auth.spot.robot");
  EXPECT_EQ(spot_sim::FindServiceEntry(entries, "robot-id")->authority(), "id.spot.robot");
  EXPECT_EQ(spot_sim::FindServiceEntry(entries, "robot-state")->authority(), "api.spot.robot");
}

TEST(Directory, DoesNotAdvertiseAnArm) {
  const auto entries = spot_sim::DefaultServiceEntries();
  EXPECT_FALSE(spot_sim::FindServiceEntry(entries, "gripper-camera-param").has_value());
  EXPECT_FALSE(spot_sim::FindServiceEntry(entries, "manipulation").has_value());
}
