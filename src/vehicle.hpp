#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/core/class_db.hpp>

using namespace godot;

class Vehicle : public Node3D {
	GDCLASS(Vehicle, Node3D)

private:
	float speed = 10.0F;

protected:
	static void _bind_methods();

public:
	void _ready() override;
	void _physics_process(double delta) override;

	void set_speed(float param);
	float get_speed() const { return speed; }
};
