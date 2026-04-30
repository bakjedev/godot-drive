#pragma once
#include "godot_cpp/classes/window.hpp"
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/classes/scene_tree.hpp>

using namespace godot;

namespace debug_ui {
template <typename... Args> inline void set(const String &fmt, Args... args) {
  Array values;
  (values.append(Variant(args)), ...);
  const auto *scene_tree =
      Object::cast_to<SceneTree>(Engine::get_singleton()->get_main_loop());
  if (!scene_tree) {
    return;
  }
  Node *overlay = scene_tree->get_root()->get_node_or_null(NodePath("DebugUi"));
  if (!overlay) {
    return;
  }
  overlay->call("set_value", fmt, values);
}

} // namespace debug_ui
// created by claude