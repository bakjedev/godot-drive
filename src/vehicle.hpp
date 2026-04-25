#pragma once
#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/core/class_db.hpp>

namespace godot {
class RigidBody3D;
class MeshInstance3D;
} // namespace godot

using namespace godot;

class Vehicle : public Node3D {
  GDCLASS(Vehicle, Node3D)

private:
  MeshInstance3D *mesh;
  RigidBody3D *rigid_body;

  Vector<Vector3> ray_positions;

  float suspension_travel = 0.2F;

protected:
  static void _bind_methods();

public:
  void _ready() override;
  void _physics_process(double delta) override;

  void set_suspension_travel(float param);
  [[nodiscard]] float get_suspension_travel() const {
    return suspension_travel;
  }
};
