#pragma once
#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/core/class_db.hpp>

namespace godot {
class RigidBody3D;
class MeshInstance3D;
} // namespace godot

using namespace godot;

struct Wheel {
  Vector3 position;
  float previous_travel = 0.0F;
};

class Vehicle : public Node3D {
  GDCLASS(Vehicle, Node3D)

private:
  MeshInstance3D *mesh;
  RigidBody3D *rigid_body;

  float suspension_travel = 0.2F;
  float suspension_rest = 0.5F;

  Vector<Wheel> wheels;

  static void setup_debug_draw();
  void debug_draw() const;

protected:
  static void _bind_methods();

public:
  void _ready() override;
  void _process(double delta) override;
  void _physics_process(double delta) override;

  void set_suspension_travel(float param);
  void set_suspension_rest(float param);
  [[nodiscard]] float get_suspension_travel() const {
    return suspension_travel;
  }
  [[nodiscard]] float get_suspension_rest() const { return suspension_rest; }
};
