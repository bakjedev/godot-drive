#pragma once
#include <godot_cpp/classes/curve.hpp>
#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/core/class_db.hpp>

namespace godot {
class RigidBody3D;
class MeshInstance3D;
} // namespace godot

using namespace godot;

struct Wheel {
  Vector3 position;
  float compression = 0.0F;
  bool in_air = true;
  float angular_velocity = 0.0F;
};

class Vehicle : public Node3D {
  GDCLASS(Vehicle, Node3D)

private:
  MeshInstance3D *mesh;
  RigidBody3D *rigid_body;
  Vector3 center_of_gravity;

  float wheel_radius = 0.35F;
  float wheel_mass = 20.0F;

  float suspension_rest = 0.1F;
  float suspension_stiffness = 25000.0F;
  float suspension_damping = 4000.0F;

  Ref<Curve> load_sensitivity_curve;
  float cornering_stiffness = 5000.0F;
  float longitudinal_stiffness = 3000.0F;

  Vector<Wheel> wheels;

  bool ready = false;
  static constexpr float ray_epsilon = 0.001F;

  bool setup_center_of_gravity();

protected:
  static void _bind_methods();

public:
  void _ready() override;
  void _process(double delta) override;
  void _physics_process(double delta) override;

  void set_wheel_radius(float param);
  [[nodiscard]] float get_wheel_radius() const { return wheel_radius; }

  void set_suspension_rest(float param);
  [[nodiscard]] float get_suspension_rest() const { return suspension_rest; }

  void set_suspension_stiffness(float param);
  [[nodiscard]] float get_suspension_stiffness() const {
    return suspension_stiffness;
  }

  void set_suspension_damping(float param);
  [[nodiscard]] float get_suspension_damping() const {
    return suspension_damping;
  }

  void set_load_sensitivity_curve(const Ref<Curve> &curve);
  [[nodiscard]] Ref<Curve> get_load_sensitivity_curve() const {
    return load_sensitivity_curve;
  }
};
