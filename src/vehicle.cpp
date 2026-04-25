#include "vehicle.hpp"

#include "godot_cpp/classes/engine.hpp"
#include "godot_cpp/classes/mesh_instance3d.hpp"
#include "godot_cpp/classes/physics_direct_space_state3d.hpp"
#include "godot_cpp/classes/physics_ray_query_parameters3d.hpp"
#include "godot_cpp/classes/rigid_body3d.hpp"
#include "godot_cpp/classes/world3d.hpp"

void Vehicle::_bind_methods() {
  ClassDB::bind_method(D_METHOD("get_suspension_travel"),
                       &Vehicle::get_suspension_travel);
  ClassDB::bind_method(D_METHOD("set_suspension_travel", "travel"),
                       &Vehicle::set_suspension_travel);
  ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "suspension_travel"),
               "set_suspension_travel", "get_suspension_travel");
}

void Vehicle::_ready() {
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

  ray_positions.push_back(aabb.position);
  ray_positions.push_back(aabb.position + Vector3(aabb.size.x, 0, 0));
  ray_positions.push_back(aabb.position + Vector3(0, 0, aabb.size.z));
  ray_positions.push_back(aabb.position + Vector3(aabb.size.x, 0, aabb.size.z));

  Object *debug_draw = Engine::get_singleton()->get_singleton("DebugDraw3D");
  const Variant cfg = debug_draw->call("scoped_config");
  auto *cfg_obj = cast_to<Object>(cfg);
  cfg_obj->call("set_thickness", 0.005F);
  cfg_obj->call("set_text_outline_size", 4);
  cfg_obj->call("set_no_depth_test", true);

  UtilityFunctions::print("Vehicle ready");
}

void Vehicle::_physics_process(double delta) {
  const Transform3D transform = rigid_body->get_global_transform();
  Object *debug_draw = Engine::get_singleton()->get_singleton("DebugDraw3D");

  const auto down = Vector3(0, -1, 0);
  PhysicsDirectSpaceState3D *space = get_world_3d()->get_direct_space_state();
  TypedArray<RID> exclude;
  exclude.push_back(rigid_body);

  for (const Vector3 &position : ray_positions) {
    const auto start = transform.xform(position);
    const auto end = start + down * suspension_travel;
    const auto query = PhysicsRayQueryParameters3D::create(start, end);
    query->set_exclude(exclude);

    const Dictionary result = space->intersect_ray(query);

    float travel = suspension_travel;
    if (result.is_empty()) {
      if (debug_draw) {
        debug_draw->call("draw_line", start, end, Color(1, 0, 0));
      }
    } else {
      const Vector3 hit_position = result["position"];
      const float distance = hit_position.distance_to(start);
      travel = distance / suspension_travel;
      if (debug_draw) {
        debug_draw->call("draw_line", start, hit_position, Color(0, 1, 0));
        debug_draw->call("draw_text", start,
                         String::num(travel).pad_decimals(2), 16,
                         Color(1, 1, 1));
      }
    }
  }
}

void Vehicle::set_suspension_travel(const float param) {
  suspension_travel = param;
}
