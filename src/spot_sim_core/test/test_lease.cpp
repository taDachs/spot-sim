#include <gtest/gtest.h>

#include "spot_sim_core/lease.hpp"

using namespace std::chrono_literals;
namespace api = bosdyn::api;

namespace {
const spot_sim::WallTime kT0{};

api::AcquireLeaseRequest AcquireRequest(const std::string& client, const std::string& resource = "body") {
  api::AcquireLeaseRequest request;
  request.mutable_header()->set_client_name(client);
  request.set_resource(resource);
  return request;
}

api::TakeLeaseRequest TakeRequest(const std::string& client) {
  api::TakeLeaseRequest request;
  request.mutable_header()->set_client_name(client);
  request.set_resource("body");
  return request;
}
}  // namespace

TEST(Lease, AcquireGrantsBodyLease) {
  spot_sim::LeaseManager leases;
  const auto response = leases.Acquire(AcquireRequest("a"), kT0);
  ASSERT_EQ(response.status(), api::AcquireLeaseResponse::STATUS_OK);
  EXPECT_EQ(response.lease().resource(), "body");
  EXPECT_EQ(response.lease().epoch(), leases.epoch());
  EXPECT_EQ(response.lease_owner().client_name(), "a");
  EXPECT_EQ(leases.Use(response.lease(), kT0).status(), api::LeaseUseResult::STATUS_OK);
}

TEST(Lease, AcquireFailsWhileClaimedButSucceedsOnceStale) {
  spot_sim::LeaseManager leases(10s);
  ASSERT_EQ(leases.Acquire(AcquireRequest("a"), kT0).status(), api::AcquireLeaseResponse::STATUS_OK);
  EXPECT_EQ(leases.Acquire(AcquireRequest("b"), kT0 + 5s).status(),
            api::AcquireLeaseResponse::STATUS_RESOURCE_ALREADY_CLAIMED);
  EXPECT_EQ(leases.Acquire(AcquireRequest("b"), kT0 + 11s).status(), api::AcquireLeaseResponse::STATUS_OK);
}

TEST(Lease, RetainKeepsLeaseFresh) {
  spot_sim::LeaseManager leases(10s);
  const auto lease = leases.Acquire(AcquireRequest("a"), kT0).lease();
  api::RetainLeaseRequest retain;
  *retain.mutable_lease() = lease;
  for (auto t = 5s; t <= 30s; t += 5s) {
    EXPECT_EQ(leases.Retain(retain, kT0 + t).lease_use_result().status(), api::LeaseUseResult::STATUS_OK);
  }
  EXPECT_EQ(leases.Acquire(AcquireRequest("b"), kT0 + 31s).status(),
            api::AcquireLeaseResponse::STATUS_RESOURCE_ALREADY_CLAIMED);
}

TEST(Lease, TakeMakesPreviousLeaseOlder) {
  spot_sim::LeaseManager leases;
  const auto first = leases.Acquire(AcquireRequest("a"), kT0).lease();
  const auto taken = leases.Take(TakeRequest("b"), kT0);
  ASSERT_EQ(taken.status(), api::TakeLeaseResponse::STATUS_OK);
  EXPECT_EQ(leases.Use(first, kT0).status(), api::LeaseUseResult::STATUS_OLDER);
  EXPECT_EQ(leases.Use(taken.lease(), kT0).status(), api::LeaseUseResult::STATUS_OK);
}

TEST(Lease, BodyLeaseAndSubleaseCoverMobility) {
  spot_sim::LeaseManager leases;
  auto lease = leases.Acquire(AcquireRequest("a"), kT0).lease();
  lease.set_resource("mobility");
  lease.add_sequence(1);  // sublease
  EXPECT_EQ(leases.Use(lease, kT0).status(), api::LeaseUseResult::STATUS_OK);
}

TEST(Lease, RejectsForeignEpochAndUnknownResource) {
  spot_sim::LeaseManager leases;
  auto lease = leases.Acquire(AcquireRequest("a"), kT0).lease();
  auto wrong_epoch = lease;
  wrong_epoch.set_epoch("other");
  EXPECT_EQ(leases.Use(wrong_epoch, kT0).status(), api::LeaseUseResult::STATUS_WRONG_EPOCH);
  EXPECT_EQ(leases.Use(api::Lease{}, kT0).status(), api::LeaseUseResult::STATUS_INVALID_LEASE);
  EXPECT_EQ(leases.Acquire(AcquireRequest("a", "arm"), kT0).status(), api::AcquireLeaseResponse::STATUS_INVALID_RESOURCE);
}

TEST(Lease, ReturnReleasesAndRevokes) {
  spot_sim::LeaseManager leases;
  const auto lease = leases.Acquire(AcquireRequest("a"), kT0).lease();
  api::ReturnLeaseRequest request;
  *request.mutable_lease() = lease;
  EXPECT_EQ(leases.Return(request, kT0).status(), api::ReturnLeaseResponse::STATUS_OK);
  EXPECT_EQ(leases.Use(lease, kT0).status(), api::LeaseUseResult::STATUS_REVOKED);
  EXPECT_EQ(leases.Return(request, kT0).status(), api::ReturnLeaseResponse::STATUS_NOT_ACTIVE_LEASE);
  EXPECT_EQ(leases.Acquire(AcquireRequest("b"), kT0).status(), api::AcquireLeaseResponse::STATUS_OK);
}

TEST(Lease, ListReportsTreeOwnerAndStaleness) {
  spot_sim::LeaseManager leases(10s);
  leases.Acquire(AcquireRequest("a"), kT0);
  auto list = leases.List(kT0 + 1s);
  EXPECT_EQ(list.resource_tree().resource(), "all-leases");
  ASSERT_EQ(list.resources_size(), 2);
  EXPECT_EQ(list.resources(0).lease_owner().client_name(), "a");
  EXPECT_FALSE(list.resources(0).is_stale());
  EXPECT_TRUE(leases.List(kT0 + 20s).resources(0).is_stale());
}
