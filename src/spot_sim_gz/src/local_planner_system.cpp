// Local planner for the simulated Spot: obstacle avoidance on walking
// velocities and walking to goals, on a robot-centred 2.5D map of the world.
//
// Contract (see spot_sim_gz/components.hpp): attached to the robot model. It
// reads SpotNavigationRequest (from SpotApiSystem) and, while the
// SpotLocomotionCommand's mode is kVelocity, rewrites its v_x / v_y / v_rot
// with the planned velocity before the base reads it. Everything else in the
// command (power, sit / stand, posture) is left alone. It reports progress in
// SpotNavigationStatus.
//
// The map comes from the world's collision geometry (ground truth): boxes,
// spheres, cylinders, capsules and planes of every other model; static models
// are drawn once per map cell, movable ones every cycle. The planning itself
// (spot_sim_planning::LocalPlanner: traversability, distance and cost-to-go
// fields, MPPI) runs on a worker thread every `period` of simulation time:
// started from the state at time t, its result is applied at t + period, and
// the simulation waits for it if needed. So results don't depend on CPU speed
// or the real-time factor.

#include <chrono>
#include <cmath>
#include <future>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/msgs/marker.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/Geometry.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Static.hh>
#include <gz/transport/Node.hh>
#include <sdf/Box.hh>
#include <sdf/Capsule.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Ellipsoid.hh>
#include <sdf/Element.hh>
#include <sdf/Geometry.hh>
#include <sdf/Plane.hh>
#include <sdf/Sphere.hh>

#include <spot_sim_planning/local_planner.hpp>

#include "spot_sim_gz/components.hpp"

namespace spot_sim_gz {

namespace components = gz::sim::components;
namespace planning = spot_sim::planning;
using spot_sim::NavigationRequest;
using spot_sim::NavigationStatus;

namespace {

/// A collision of another model, in its own frame.
struct Collider {
  gz::sim::Entity entity = gz::sim::kNullEntity;
  planning::Shape shape;     ///< geometry; position / orientation relative to the collision frame
  gz::math::Pose3d offset;   ///< extra pose of the shape in the collision frame (planes)
};

std::optional<Collider> MakeCollider(gz::sim::Entity entity, const sdf::Geometry& geometry, const std::string& name) {
  Collider c;
  c.entity = entity;
  planning::Shape& s = c.shape;
  switch (geometry.Type()) {
    case sdf::GeometryType::BOX: {
      const auto size = geometry.BoxShape()->Size();
      s.kind = planning::Shape::Kind::kBox;
      s.size = {size.X(), size.Y(), size.Z()};
      break;
    }
    case sdf::GeometryType::SPHERE:
      s.kind = planning::Shape::Kind::kSphere;
      s.size = {geometry.SphereShape()->Radius(), 0.0, 0.0};
      break;
    case sdf::GeometryType::CYLINDER:
      s.kind = planning::Shape::Kind::kCylinder;
      s.size = {geometry.CylinderShape()->Radius(), 0.0, geometry.CylinderShape()->Length()};
      break;
    case sdf::GeometryType::CAPSULE:
      s.kind = planning::Shape::Kind::kCapsule;
      s.size = {geometry.CapsuleShape()->Radius(), 0.0, geometry.CapsuleShape()->Length()};
      break;
    case sdf::GeometryType::ELLIPSOID: {
      const auto radii = geometry.EllipsoidShape()->Radii();
      s.kind = planning::Shape::Kind::kBox;  // its bounding box
      s.size = {2.0 * radii.X(), 2.0 * radii.Y(), 2.0 * radii.Z()};
      break;
    }
    case sdf::GeometryType::PLANE: {
      const auto* plane = geometry.PlaneShape();
      s.kind = planning::Shape::Kind::kPlane;
      s.size = {plane->Size().X(), plane->Size().Y(), 0.0};
      // Local z along the plane's normal.
      gz::math::Quaterniond rotation;
      rotation.SetFrom2Axes(gz::math::Vector3d::UnitZ, plane->Normal());
      c.offset = gz::math::Pose3d(gz::math::Vector3d::Zero, rotation);
      break;
    }
    default:
      gzwarn << "[LocalPlannerSystem] collision '" << name
             << "': geometry type not supported by the planner's map (meshes, heightmaps); ignored\n";
      return std::nullopt;
  }
  return c;
}

planning::Shape Placed(const Collider& c, const gz::math::Pose3d& world_T_collision) {
  const gz::math::Pose3d pose = world_T_collision * c.offset;
  planning::Shape s = c.shape;
  s.position = {pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z()};
  s.orientation = {pose.Rot().W(), pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z()};
  return s;
}

NavigationStatus::State ToState(planning::LocalPlannerState state) {
  switch (state) {
    case planning::LocalPlannerState::kMoving:
      return NavigationStatus::State::kMoving;
    case planning::LocalPlannerState::kNearGoal:
      return NavigationStatus::State::kNearGoal;
    case planning::LocalPlannerState::kAtGoal:
      return NavigationStatus::State::kAtGoal;
    case planning::LocalPlannerState::kBlocked:
      return NavigationStatus::State::kBlocked;
  }
  return NavigationStatus::State::kIdle;
}

const char* StateName(NavigationStatus::State state) {
  switch (state) {
    case NavigationStatus::State::kIdle:
      return "idle";
    case NavigationStatus::State::kMoving:
      return "moving";
    case NavigationStatus::State::kNearGoal:
      return "near goal";
    case NavigationStatus::State::kAtGoal:
      return "at goal";
    case NavigationStatus::State::kBlocked:
      return "blocked";
  }
  return "?";
}

double Seconds(std::chrono::steady_clock::duration d) { return std::chrono::duration<double>(d).count(); }

}  // namespace

class LocalPlannerSystem : public gz::sim::System,
                           public gz::sim::ISystemConfigure,
                           public gz::sim::ISystemConfigurePriority,
                           public gz::sim::ISystemPreUpdate,
                           public gz::sim::ISystemPostUpdate {
 public:
  /// After SpotApiSystem (-100) wrote the command, before the base (0) reads it.
  gz::sim::System::PriorityType ConfigurePriority() override { return -50; }

  void Configure(const gz::sim::Entity& entity, const std::shared_ptr<const sdf::Element>& sdf,
                 gz::sim::EntityComponentManager& ecm, gz::sim::EventManager& /*events*/) override {
    model_ = gz::sim::Model(entity);
    if (!model_.Valid(ecm)) {
      gzerr << "[LocalPlannerSystem] must be attached to a model\n";
      return;
    }
    const std::string body = sdf->Get<std::string>("body_link", "body").first;
    body_link_ = model_.LinkByName(ecm, body);
    if (body_link_ == gz::sim::kNullEntity) {
      gzerr << "[LocalPlannerSystem] model has no link '" << body << "'\n";
      return;
    }
    planning::LocalPlannerConfig config;
    config.mppi.samples = sdf->Get<int>("samples", config.mppi.samples).first;
    config.obstacle_ceiling = sdf->Get<double>("obstacle_ceiling", config.obstacle_ceiling).first;
    period_ = config.mppi.dt;
    markers_ = sdf->Get<bool>("debug_markers", false).first;
    planner_ = std::make_unique<planning::LocalPlanner>(config);
    valid_ = true;
    gzmsg << "[LocalPlannerSystem] local planner active on model '" << model_.Name(ecm) << "'"
          << (markers_ ? " (debug markers on /marker)" : "") << "\n";
  }

  void PreUpdate(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm) override {
    if (!valid_) return;
    if (!ecm.Component<components::SpotNavigationStatus>(model_.Entity())) {
      ecm.CreateComponent(model_.Entity(), components::SpotNavigationStatus(NavigationStatus{}));
    }
    WatchGeometry(ecm);
    if (info.paused) return;

    const auto* request_component = ecm.Component<components::SpotNavigationRequest>(model_.Entity());
    const auto* command_component = ecm.Component<components::SpotLocomotionCommand>(model_.Entity());
    const NavigationRequest request = request_component ? request_component->Data() : NavigationRequest{};
    const spot_sim::LocomotionCommand raw = command_component ? command_component->Data() : spot_sim::LocomotionCommand{};
    spot_sim::LocomotionCommand command = raw;
    const bool active = Active(request, command);
    const double now = Seconds(info.simTime);

    // Collect the result that is due.
    if (pending_ && now >= pending_->apply_time - 1e-9) {
      const auto started = std::chrono::steady_clock::now();
      const planning::LocalPlannerOutput out = pending_->result.get();
      waited_ += Seconds(std::chrono::steady_clock::now() - started);
      if (active && pending_->key == Key(request)) {
        velocity_ = out.command;
        planned_for_ = pending_->key;
        status_ = NavigationStatus{ToState(out.state), request.sequence};
        if (markers_) PublishMarkers(out, pending_->robot_z);
      }
      pending_.reset();
    }

    if (!active) {
      velocity_ = planning::Velocity2{};
      planned_for_.reset();
      status_ = NavigationStatus{NavigationStatus::State::kIdle, request.sequence};
    } else {
      if (planned_for_ != Key(request)) {
        // Nothing planned for this request yet: stand still until the first plan.
        velocity_ = planning::Velocity2{};
        status_ = NavigationStatus{NavigationStatus::State::kMoving, request.sequence};
      }
      command.v_x = velocity_.vx;
      command.v_y = velocity_.vy;
      command.v_rot = velocity_.wz;
      ecm.SetComponentData<components::SpotLocomotionCommand>(model_.Entity(), command);
    }
    if (status_ != logged_status_) {
      gzmsg << "[LocalPlannerSystem] " << StateName(status_.state)
            << (request.kind == NavigationRequest::Kind::kGoal
                    ? " (goal " + std::to_string(request.sequence) + " at odom " + std::to_string(request.goal_x) +
                          ", " + std::to_string(request.goal_y) + ")"
                    : std::string())
            << "\n";
      logged_status_ = status_;
    }
    ecm.SetComponentData<components::SpotNavigationStatus>(model_.Entity(), status_);
    active_ = active;
    request_ = request;
    raw_command_ = raw;
  }

  void PostUpdate(const gz::sim::UpdateInfo& info, const gz::sim::EntityComponentManager& ecm) override {
    if (!valid_ || info.paused) return;
    const double now = Seconds(info.simTime);
    if (!active_) {
      next_plan_time_ = now;  // plan right away once active
      return;
    }
    if (pending_ || now < next_plan_time_ - 1e-9) return;

    planning::LocalPlannerInput input;
    input.time = now;
    const gz::math::Pose3d world_T_body = gz::sim::worldPose(body_link_, ecm);
    input.robot = {world_T_body.Pos().X(), world_T_body.Pos().Y(), world_T_body.Rot().Yaw()};
    input.robot_z = world_T_body.Pos().Z();
    input.current = velocity_;
    input.avoid = request_.avoid;
    input.padding = request_.padding;
    input.max_velocity = {request_.max_vx, request_.max_vy, request_.max_wz};
    input.max_accel = {raw_command_.max_accel_x, raw_command_.max_accel_y, raw_command_.max_accel_rot};
    if (request_.kind == NavigationRequest::Kind::kGoal) {
      input.mode = planning::LocalPlannerInput::Mode::kGoal;
      const gz::math::Pose3d world_T_odom(request_.odom_x, request_.odom_y, 0.0, 0.0, 0.0, request_.odom_yaw);
      const gz::math::Pose3d goal = world_T_odom * gz::math::Pose3d(request_.goal_x, request_.goal_y, 0.0, 0.0, 0.0,
                                                                    request_.goal_yaw);
      input.goal = {goal.Pos().X(), goal.Pos().Y(), goal.Rot().Yaw()};
      input.goal_sequence = request_.sequence;
      input.position_tolerance = request_.position_tolerance;
      input.yaw_tolerance = request_.yaw_tolerance;
    } else {
      input.mode = planning::LocalPlannerInput::Mode::kVelocity;
      input.commanded = {raw_command_.v_x, raw_command_.v_y, raw_command_.v_rot};
    }
    input.static_shapes = static_shapes_;
    input.static_version = static_version_;
    for (const Collider& c : movable_) {
      input.movable_shapes.push_back(Placed(c, gz::sim::worldPose(c.entity, ecm)));
    }

    Pending job;
    job.apply_time = now + period_;
    job.key = Key(request_);
    job.robot_z = input.robot_z - planner_->config().stand_height;
    planning::LocalPlanner* planner = planner_.get();
    job.result = std::async(std::launch::async, [planner, input = std::move(input)] { return planner->Step(input); });
    pending_ = std::move(job);
    next_plan_time_ = now + period_;

    if (++cycles_ % 400 == 0) {  // every 20 s of simulation time
      gzdbg << "[LocalPlannerSystem] waited " << waited_ << " s for the planner in the last 400 cycles\n";
      waited_ = 0.0;
    }
  }

 private:
  struct Pending {
    double apply_time = 0.0;
    std::pair<NavigationRequest::Kind, uint32_t> key;
    double robot_z = 0.0;  ///< feet height, for markers
    std::future<planning::LocalPlannerOutput> result;
  };

  /// What a plan is for: a goal (by sequence), or walking at velocity (any
  /// command: they keep coming while a client drives).
  static std::pair<NavigationRequest::Kind, uint32_t> Key(const NavigationRequest& r) {
    return {r.kind, r.kind == NavigationRequest::Kind::kGoal ? r.sequence : 0};
  }

  static bool Active(const NavigationRequest& request, const spot_sim::LocomotionCommand& command) {
    if (!command.motors_on || command.mode != spot_sim::LocomotionMode::kVelocity) return false;
    switch (request.kind) {
      case NavigationRequest::Kind::kNone:
        return false;
      case NavigationRequest::Kind::kVelocity:
        return request.avoid;
      case NavigationRequest::Kind::kGoal:
        return true;
    }
    return false;
  }

  /// Keeps the list of other models' collisions current: static ones as a
  /// shared shape list (a new version when it changes), movable ones as
  /// entities whose pose is read every cycle.
  void WatchGeometry(gz::sim::EntityComponentManager& ecm) {
    bool changed = !scanned_;
    ecm.EachNew<components::Collision>([&](const gz::sim::Entity&, const components::Collision*) {
      changed = true;
      return true;
    });
    ecm.EachRemoved<components::Collision>([&](const gz::sim::Entity&, const components::Collision*) {
      changed = true;
      return true;
    });
    if (!changed) return;
    scanned_ = true;
    auto statics = std::make_shared<std::vector<planning::Shape>>();
    movable_.clear();
    ecm.Each<components::Collision, components::Geometry>(
        [&](const gz::sim::Entity& entity, const components::Collision*, const components::Geometry* geometry) {
          const gz::sim::Entity model = gz::sim::topLevelModel(entity, ecm);
          if (model == model_.Entity()) return true;  // the robot itself
          const auto* name = ecm.Component<components::Name>(entity);
          auto collider = MakeCollider(entity, geometry->Data(), name ? name->Data() : "?");
          if (!collider) return true;
          const auto* is_static = ecm.Component<components::Static>(model);
          if (is_static && is_static->Data()) {
            statics->push_back(Placed(*collider, gz::sim::worldPose(entity, ecm)));
          } else {
            movable_.push_back(*collider);
          }
          return true;
        });
    static_shapes_ = std::move(statics);
    ++static_version_;
    gzmsg << "[LocalPlannerSystem] world geometry: " << static_shapes_->size() << " static and " << movable_.size()
          << " movable shapes\n";
  }

  void PublishMarkers(const planning::LocalPlannerOutput& out, double ground_z) {
    gz::msgs::Marker marker;
    marker.set_ns("spot_local_planner");
    marker.set_id(1);
    marker.set_action(gz::msgs::Marker::ADD_MODIFY);
    marker.set_type(gz::msgs::Marker::LINE_STRIP);
    marker.mutable_lifetime()->set_nsec(200'000'000);
    auto* color = marker.mutable_material()->mutable_diffuse();
    color->set_r(0.1f), color->set_g(0.8f), color->set_b(0.2f), color->set_a(1.0f);
    *marker.mutable_material()->mutable_ambient() = *color;
    for (const auto& p : out.mppi.trajectory) {
      auto* point = marker.add_point();
      point->set_x(p.x), point->set_y(p.y), point->set_z(ground_z + 0.05);
    }
    node_.Request("/marker", marker);
  }

  gz::sim::Model model_{gz::sim::kNullEntity};
  gz::sim::Entity body_link_ = gz::sim::kNullEntity;
  bool valid_ = false;
  bool markers_ = false;
  double period_ = 0.05;
  std::unique_ptr<planning::LocalPlanner> planner_;

  bool scanned_ = false;
  std::shared_ptr<const std::vector<planning::Shape>> static_shapes_;
  uint64_t static_version_ = 0;
  std::vector<Collider> movable_;

  bool active_ = false;
  NavigationRequest request_;
  spot_sim::LocomotionCommand raw_command_;
  NavigationStatus status_;
  NavigationStatus logged_status_;
  planning::Velocity2 velocity_;
  std::optional<std::pair<NavigationRequest::Kind, uint32_t>> planned_for_;  ///< velocity_ is a plan for this
  std::optional<Pending> pending_;
  double next_plan_time_ = 0.0;
  uint64_t cycles_ = 0;
  double waited_ = 0.0;  ///< wall time the simulation waited for plans [s]

  gz::transport::Node node_;
};

}  // namespace spot_sim_gz

GZ_ADD_PLUGIN(spot_sim_gz::LocalPlannerSystem, gz::sim::System, spot_sim_gz::LocalPlannerSystem::ISystemConfigure,
              spot_sim_gz::LocalPlannerSystem::ISystemConfigurePriority,
              spot_sim_gz::LocalPlannerSystem::ISystemPreUpdate, spot_sim_gz::LocalPlannerSystem::ISystemPostUpdate)
