#include "vehicle.hpp"

#include "godot_cpp/classes/engine.hpp"
#include "godot_cpp/classes/mesh_instance3d.hpp"
#include "godot_cpp/classes/physics_direct_space_state3d.hpp"
#include "godot_cpp/classes/physics_ray_query_parameters3d.hpp"
#include "godot_cpp/classes/rigid_body3d.hpp"
#include "godot_cpp/classes/world3d.hpp"

void Vehicle::setup_debug_draw() {
  Object *debug_draw = Engine::get_singleton()->get_singleton("DebugDraw3D");
  const Variant cfg = debug_draw->call("scoped_config");
  auto *cfg_obj = cast_to<Object>(cfg);
  cfg_obj->call("set_thickness", 0.005F);
  cfg_obj->call("set_text_outline_size", 4);
  cfg_obj->call("set_no_depth_test", true);
}

void Vehicle::debug_draw() const {
  Object *dd3d = Engine::get_singleton()->get_singleton("DebugDraw3D");

  if (dd3d == nullptr) {
    return;
  }

  const Transform3D transform = rigid_body->get_global_transform();
  const auto up = Vector3(0, 1, 0);
  for (const Wheel &wheel : wheels) {
    const auto start = transform.xform(wheel.position);
    const auto end = start - up * (wheel.previous_travel * suspension_travel);

    Color color = wheel.previous_travel * suspension_travel < suspension_travel
                      ? Color(0, 1, 0)
                      : Color(1, 0, 0);

    dd3d->call("draw_line", start, end, color);
    dd3d->call("draw_text", start,
               String::num(wheel.previous_travel).pad_decimals(2), 16,
               Color(1, 1, 1));
  }
}

void Vehicle::_bind_methods() {
  ClassDB::bind_method(D_METHOD("get_suspension_travel"),
                       &Vehicle::get_suspension_travel);
  ClassDB::bind_method(D_METHOD("set_suspension_travel", "travel"),
                       &Vehicle::set_suspension_travel);
  ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "suspension_travel"),
               "set_suspension_travel", "get_suspension_travel");

  ClassDB::bind_method(D_METHOD("get_suspension_rest"),
                       &Vehicle::get_suspension_rest);
  ClassDB::bind_method(D_METHOD("set_suspension_rest", "rest"),
                       &Vehicle::set_suspension_rest);
  ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "suspension_rest"),
               "set_suspension_rest", "get_suspension_rest");
}

void Vehicle::_ready() {
  wheels.resize(4);

  rigid_body = get_node<RigidBody3D>("RigidBody");
  if (rigid_body == nullptr) {
    UtilityFunctions::printerr("Vehicle couldn't find rigid body!");
    return;
  }

  mesh = rigid_body->get_node<MeshInstance3D>("Mesh");
  if (mesh == nullptr) {
    UtilityFunctions::printerr("Vehicle couldn't find mesh!");
    return;
  }
  auto aabb = mesh->get_aabb();
  aabb.position = aabb.position * mesh->get_scale();
  aabb.size = aabb.size * mesh->get_scale();

  wheels.write[0].position = aabb.position;
  wheels.write[1].position = aabb.position + Vector3(aabb.size.x, 0, 0);
  wheels.write[2].position = aabb.position + Vector3(0, 0, aabb.size.z);
  wheels.write[3].position =
      aabb.position + Vector3(aabb.size.x, 0, aabb.size.z);

  setup_debug_draw();

  UtilityFunctions::print("Vehicle ready");
}
void Vehicle::_process(double delta) { debug_draw(); }

void Vehicle::_physics_process(const double delta) {

  const Transform3D transform = rigid_body->get_global_transform();
  PhysicsDirectSpaceState3D *space = get_world_3d()->get_direct_space_state();
  TypedArray<RID> exclude;
  exclude.push_back(rigid_body);
  const auto down = Vector3(0, -1, 0);
  const float resting_position = suspension_travel * suspension_rest;

  for (Wheel &wheel : wheels) {

    const auto start = transform.xform(wheel.position);
    const auto end = start + down * suspension_travel;
    const auto query = PhysicsRayQueryParameters3D::create(start, end);
    query->set_exclude(exclude);

    const Dictionary result = space->intersect_ray(query);

    float travel = 1.0F;
    if (!result.is_empty()) {
      const Vector3 hit_position = result["position"];
      const float distance = hit_position.distance_to(start);
      travel = distance / suspension_travel;
      rigid_body->apply_force(-down * 500.0F * (1.0F - travel) *
                                  static_cast<real_t>(delta),
                              wheel.position);
    }

    wheel.previous_travel = travel;
  }
}

void Vehicle::set_suspension_travel(const float param) {
  suspension_travel = param;
}
void Vehicle::set_suspension_rest(const float param) {
  suspension_rest = param;
}
