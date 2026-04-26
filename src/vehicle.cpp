#include "vehicle.hpp"

#include "godot_cpp/classes/engine.hpp"
#include "godot_cpp/classes/mesh_instance3d.hpp"
#include "godot_cpp/classes/physics_direct_space_state3d.hpp"
#include "godot_cpp/classes/physics_ray_query_parameters3d.hpp"
#include "godot_cpp/classes/rigid_body3d.hpp"
#include "godot_cpp/classes/world3d.hpp"

bool Vehicle::setup_debug_draw() {
  Object *debug_draw = Engine::get_singleton()->get_singleton("DebugDraw3D");
  if (debug_draw == nullptr) {
    return false;
  }
  const Variant cfg = debug_draw->call("scoped_config");
  auto *cfg_obj = cast_to<Object>(cfg);
  if (cfg_obj == nullptr) {
    return false;
  }
  cfg_obj->call("set_thickness", 0.005F);
  cfg_obj->call("set_text_outline_size", 4);
  cfg_obj->call("set_no_depth_test", true);
  return true;
}

void Vehicle::debug_draw() const {
  Object *dd3d = Engine::get_singleton()->get_singleton("DebugDraw3D");

  if (dd3d == nullptr) {
    return;
  }

  const Transform3D transform = rigid_body->get_global_transform();
  const auto down = -transform.basis.get_column(1);
  for (const Wheel &wheel : wheels) {
    const auto start = transform.xform(wheel.position);
    const auto end = start + down * (suspension_rest + wheel_radius);

    const Color color = wheel.in_air ? Color(1, 0, 0) : Color(0, 1, 0);

    dd3d->call("draw_line", start, end, color);
    dd3d->call("draw_text", start,
               String::num(wheel.compression).pad_decimals(2), 16,
               Color(1, 1, 1));
  }
}

bool Vehicle::setup_center_of_gravity() {
  auto *cg_node = rigid_body->get_node<Node3D>("CG");
  if (cg_node == nullptr) {
    return false;
  }
  center_of_gravity = cg_node->get_position();
  rigid_body->set_center_of_mass(center_of_gravity);
  return true;
}

void Vehicle::_bind_methods() {
  // wheel radius
  ClassDB::bind_method(D_METHOD("get_wheel_radius"),
                       &Vehicle::get_wheel_radius);
  ClassDB::bind_method(D_METHOD("set_wheel_radius", "radius"),
                       &Vehicle::set_wheel_radius);
  ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "wheel_radius"), "set_wheel_radius",
               "get_wheel_radius");

  // suspension rest
  ClassDB::bind_method(D_METHOD("get_suspension_rest"),
                       &Vehicle::get_suspension_rest);
  ClassDB::bind_method(D_METHOD("set_suspension_rest", "rest"),
                       &Vehicle::set_suspension_rest);
  ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "suspension_rest"),
               "set_suspension_rest", "get_suspension_rest");

  // suspension stiffness
  ClassDB::bind_method(D_METHOD("get_suspension_stiffness"),
                       &Vehicle::get_suspension_stiffness);
  ClassDB::bind_method(D_METHOD("set_suspension_stiffness", "stiffness"),
                       &Vehicle::set_suspension_stiffness);
  ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "suspension_stiffness"),
               "set_suspension_stiffness", "get_suspension_stiffness");

  // suspension damping
  ClassDB::bind_method(D_METHOD("get_suspension_damping"),
                       &Vehicle::get_suspension_damping);
  ClassDB::bind_method(D_METHOD("set_suspension_damping", "damping"),
                       &Vehicle::set_suspension_damping);
  ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "suspension_damping"),
               "set_suspension_damping", "get_suspension_damping");
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

  if (!setup_center_of_gravity()) {
    UtilityFunctions::printerr("Vehicle couldn't setup center of gravity!");
    return;
  }

  if (!setup_debug_draw()) {
    UtilityFunctions::printerr("Vehicle couldn't setup debug drawing!");
    return;
  }

  UtilityFunctions::print("Vehicle ready");
}
void Vehicle::_process(double delta) { debug_draw(); }

void Vehicle::_physics_process(const double delta) {

  const Transform3D transform = rigid_body->get_global_transform();
  PhysicsDirectSpaceState3D *space = get_world_3d()->get_direct_space_state();
  TypedArray<RID> exclude;
  exclude.push_back(rigid_body);
  const auto up = transform.basis.get_column(1);

  for (Wheel &wheel : wheels) {
    const Vector3 wheel_world = transform.xform(wheel.position);

    const auto start = wheel_world;
    const auto end = start - up * (suspension_rest + wheel_radius);
    const auto query = PhysicsRayQueryParameters3D::create(start, end);
    query->set_exclude(exclude);

    const Dictionary result = space->intersect_ray(query);

    if (!result.is_empty()) {
      wheel.in_air = false;
      const Vector3 hit_position = result["position"];
      const float distance = hit_position.distance_to(start);

      wheel.compression = suspension_rest - (distance - wheel_radius);

      const Vector3 body_velocity = rigid_body->get_linear_velocity();
      const Vector3 wheel_offset =
          wheel_world - rigid_body->get_global_position();
      const Vector3 point_velocity =
          body_velocity +
          rigid_body->get_angular_velocity().cross(wheel_offset);

      const float compression_rate = up.dot(point_velocity);

      auto damped_harmonic_oscillator = [](const float k, const float x,
                                           const float c, const float v) {
        return -(k * x) - (c * v);
      };

      const float suspension = Math::max(
          0.0F,
          damped_harmonic_oscillator(suspension_stiffness, -wheel.compression,
                                     suspension_damping, compression_rate));

      rigid_body->apply_force(up * suspension, wheel_offset);
    } else {
      wheel.in_air = true;
      wheel.compression = 0.0F;
    }
  }
}

void Vehicle::set_wheel_radius(const float param) { wheel_radius = param; }

void Vehicle::set_suspension_rest(const float param) {
  suspension_rest = param;
}

void Vehicle::set_suspension_stiffness(const float param) {
  suspension_stiffness = param;
}

void Vehicle::set_suspension_damping(const float param) {
  suspension_damping = param;
}
