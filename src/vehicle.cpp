#include "vehicle.hpp"

void Vehicle::_bind_methods() {
	ClassDB::bind_method(D_METHOD("get_speed"), &Vehicle::get_speed);
	ClassDB::bind_method(D_METHOD("set_speed", "speed"), &Vehicle::set_speed);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "speed"), "set_speed", "get_speed");
}

void Vehicle::_ready() {
	UtilityFunctions::print("Vehicle ready");
}

void Vehicle::_physics_process(double delta) {
	UtilityFunctions::print("Vehicle update");
}

void Vehicle::set_speed(float param) { speed = param; }
