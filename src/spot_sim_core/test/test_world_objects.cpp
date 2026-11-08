#include <gtest/gtest.h>

#include "spot_sim_core/world_objects.hpp"

using namespace std::chrono_literals;
namespace api = bosdyn::api;
using Mutate = api::MutateWorldObjectRequest;

namespace {
api::MutateWorldObjectRequest Mutation(Mutate::Action action, const std::string& name, int32_t id = 0) {
  api::MutateWorldObjectRequest request;
  request.mutable_mutation()->set_action(action);
  auto* object = request.mutable_mutation()->mutable_object();
  object->set_name(name);
  object->set_id(id);
  object->add_drawable_properties()->mutable_sphere()->set_radius(0.1);
  return request;
}

std::vector<std::string> Names(const api::ListWorldObjectResponse& response) {
  std::vector<std::string> names;
  for (const auto& object : response.world_objects()) names.push_back(object.name());
  return names;
}
}  // namespace

TEST(WorldObjects, AddChangeDelete) {
  spot_sim::WorldObjects objects;
  const auto added = objects.Mutate(Mutation(Mutate::ACTION_ADD, "a"), 1s);
  ASSERT_EQ(added.status(), api::MutateWorldObjectResponse::STATUS_OK);
  EXPECT_GT(added.mutated_object_id(), 0);
  objects.Mutate(Mutation(Mutate::ACTION_ADD, "b"), 1s);
  EXPECT_EQ(Names(objects.List({}, 2s)), (std::vector<std::string>{"a", "b"}));

  EXPECT_EQ(objects.Mutate(Mutation(Mutate::ACTION_CHANGE, "a2", added.mutated_object_id()), 2s).status(),
            api::MutateWorldObjectResponse::STATUS_OK);
  EXPECT_EQ(Names(objects.List({}, 2s)), (std::vector<std::string>{"a2", "b"}));

  EXPECT_EQ(objects.Mutate(Mutation(Mutate::ACTION_DELETE, "", added.mutated_object_id()), 3s).status(),
            api::MutateWorldObjectResponse::STATUS_OK);
  EXPECT_EQ(Names(objects.List({}, 3s)), (std::vector<std::string>{"b"}));

  EXPECT_EQ(objects.Mutate(Mutation(Mutate::ACTION_DELETE, "", 999), 3s).status(),
            api::MutateWorldObjectResponse::STATUS_INVALID_MUTATION_ID);
  EXPECT_EQ(objects.Mutate(Mutation(Mutate::ACTION_UNKNOWN, "x"), 3s).status(),
            api::MutateWorldObjectResponse::STATUS_INVALID_WORLD_OBJECT);
}

TEST(WorldObjects, ListFilters) {
  spot_sim::WorldObjects objects;
  objects.Mutate(Mutation(Mutate::ACTION_ADD, "old"), 1s);
  auto tag = Mutation(Mutate::ACTION_ADD, "tag");
  tag.mutable_mutation()->mutable_object()->clear_drawable_properties();
  tag.mutable_mutation()->mutable_object()->mutable_apriltag_properties()->set_tag_id(3);
  objects.Mutate(tag, 5s);

  api::ListWorldObjectRequest by_type;
  by_type.add_object_type(api::WORLD_OBJECT_APRILTAG);
  EXPECT_EQ(Names(objects.List(by_type, 6s)), (std::vector<std::string>{"tag"}));

  api::ListWorldObjectRequest by_time;
  *by_time.mutable_timestamp_filter() = spot_sim::ToTimestamp(2s);
  EXPECT_EQ(Names(objects.List(by_time, 6s)), (std::vector<std::string>{"tag"}));
}

TEST(WorldObjects, ExpireAfterLifetime) {
  spot_sim::WorldObjects objects;
  auto request = Mutation(Mutate::ACTION_ADD, "short");
  *request.mutable_mutation()->mutable_object()->mutable_object_lifetime() = spot_sim::ToDuration(2s);
  objects.Mutate(request, 10s);
  objects.Mutate(Mutation(Mutate::ACTION_ADD, "forever"), 10s);
  EXPECT_EQ(objects.List({}, 11s).world_objects_size(), 2);
  EXPECT_EQ(Names(objects.List({}, 13s)), (std::vector<std::string>{"forever"}));
}
