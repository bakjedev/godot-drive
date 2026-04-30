#include "vehicle.hpp"

#include "debug_ui.hpp"
#include "godot_cpp/classes/engine.hpp"
#include "godot_cpp/classes/mesh_instance3d.hpp"
#include "godot_cpp/classes/physics_direct_space_state3d.hpp"
#include "godot_cpp/classes/physics_ray_query_parameters3d.hpp"
#include "godot_cpp/classes/rigid_body3d.hpp"
#include "godot_cpp/classes/world3d.hpp"

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

  // load sensitivity curve
  ClassDB::bind_method(D_METHOD("get_load_sensitivity_curve"),
                       &Vehicle::get_load_sensitivity_curve);
  ClassDB::bind_method(
      D_METHOD("set_load_sensitivity_curve", "load_sensitivity"),
      &Vehicle::set_load_sensitivity_curve);
  ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "load_sensitivity_curve",
                            PROPERTY_HINT_RESOURCE_TYPE, "Curve"),
               "set_load_sensitivity_curve", "get_load_sensitivity_curve");
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

  if (load_sensitivity_curve.is_null()) {
    UtilityFunctions::printerr("Vehicle doesn't have load sensitivity curve!");
    return;
  }

  UtilityFunctions::print("Vehicle ready");
  ready = true;
}

void Vehicle::_process(double delta) {
  if (!ready) {
    return;
  }
  for (auto i = 0; i < wheels.size(); i++) {
    const auto &wheel = wheels.get(i);
    debug_ui::set("wheel " + String::num(i + 1, 0) +
                      ":\nangular_velocity: %+5.2f\ncompression: %+5.2f\n",
                  wheel.angular_velocity, wheel.compression);
  }
}

void Vehicle::_physics_process(const double delta) {
  if (!ready) {
    return;
  }

  const Transform3D transform = rigid_body->get_global_transform();
  const Vector3 body_velocity = rigid_body->get_linear_velocity();
  PhysicsDirectSpaceState3D *space = get_world_3d()->get_direct_space_state();
  TypedArray<RID> exclude;
  exclude.push_back(rigid_body);
  const auto up = transform.basis.get_column(1);
  const auto forward = -transform.basis.get_column(2);
  const auto right = transform.basis.get_column(0);

  const float gravity = rigid_body->get_gravity().length();
  const float mass = rigid_body->get_mass();
  const float nominal_wheel_load = mass * gravity / 4.0F;

  auto damped_harmonic_oscillator = [](const float k, const float x,
                                       const float c, const float v) {
    return -(k * x) - (c * v);
  };

  // r = how far along are we to total saturation
  auto brush_curve = [](const float r) {
    if (r >= 1.0F)
      return 1.0F;
    return (3.0F * r) - (3.0F * r * r) + (r * r * r);
  };

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
      const Vector3 wheel_offset =
          wheel_world - rigid_body->get_global_position();
      const Vector3 point_velocity =
          body_velocity +
          rigid_body->get_angular_velocity().cross(wheel_offset);

      const float compression_rate = up.dot(point_velocity);

      const float suspension =
          Math::max(0.0F, damped_harmonic_oscillator(
                              suspension_stiffness, -wheel.compression,
                              suspension_damping, compression_rate)); // Fz

      rigid_body->apply_force(up * suspension, wheel_offset);

      const float longitudinal = forward.dot(point_velocity);
      const float lateral = right.dot(point_velocity);

      const float slip_angle =
          Math::atan2(lateral, Math::abs(longitudinal) + 0.1f);

      const float slip_ratio =
          ((wheel.angular_velocity * wheel_radius) - longitudinal) /
          Math::max(Math::abs(longitudinal), 1.0F);

      const float load_ratio = suspension / nominal_wheel_load;
      const float mu =
          load_sensitivity_curve->sample(load_ratio); // friction coefficient
      const float max_force = mu * suspension;

      float lateral_force = 0.0F;
      float longitudinal_force = 0.0F;
      if (max_force > 0.0F) {
        // how much slipping on each axis
        const float sx =
            (longitudinal_stiffness * slip_ratio) / (3.0F * max_force);
        const float sy =
            (cornering_stiffness * Math::tan(slip_angle)) / (3.0F * max_force);
        const float sigma = Math::sqrt(sx * sx + sy * sy); // combined slip

        const float force_magnitude = max_force * brush_curve(sigma);

        if (!Math::is_zero_approx(sigma)) {
          longitudinal_force = force_magnitude * (sx / sigma);
          lateral_force = -force_magnitude * (sy / sigma);
        }
      }

      const Vector3 contact_offset =
          hit_position - rigid_body->get_global_position();

      rigid_body->apply_force(
          forward * longitudinal_force + right * lateral_force, contact_offset);

      const float inertia = 0.5F * wheel_mass * wheel_radius * wheel_radius;
      const float torque = -longitudinal_force * wheel_radius;
      wheel.angular_velocity += (torque / inertia) * static_cast<float>(delta);
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

void Vehicle::set_load_sensitivity_curve(const Ref<Curve> &curve) {
  load_sensitivity_curve = curve;
}
