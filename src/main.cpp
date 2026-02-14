
#include "physics/math/math.hpp"
#include "physicsAcc/API/World.hpp"
#include "physicsAcc/Utils/VisualsDebugWindow.hpp"

int main(int argc, char *argv[]) {
  auto world = World(true);

  size_t body_1 = world.create_body(BodyParams{
      .type = BodyType::STATIC,
      .orientation = ti::quat_from_axis_angle({1.0, 1.0, 1.0}, 0.75)});

  size_t body_2 = world.create_body(BodyParams(
      {.mass = 1.0,
       .inertia_tensor = mat3{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 500.0},
       .position = {-5.0, 0.0, 0.0}}));

  size_t collider_1 = world.create_collider(ColliderParams{
      .body_id = body_1,
      .model_type = ColliderShape::SPHERE,
      .radius = 1.0,
  });

  size_t collider_2 =
      world.create_collider(ColliderParams{.body_id = body_2,
                                           .model_type = ColliderShape::BOX,
                                           .half_extents = {0.4, 0.3, 0.2}});

  size_t joint_a = world.create_revolute_joint(RevoluteJointParams{
      .body_1 = body_1,
      .body_2 = body_2,
      .aligned_axis = {0.0, 0.0, 1.0},
      .limit_axis = {0.0, 1.0, 0.0},
      .r_1 = {-5.0, 0.0, 5.0},
      .actuation_type = JointActuationType::FREE,
  });

  // We can inject GUIS into the visualizer
  // if (world.visualizer) {
  //   DebugGUIHandler debug_gui_handler;
  //   world.visualizer->visualizer.add_gui("Physics Debugger", [&]() {
  //     debug_gui_handler.render_debug_uis(world);
  //   });
  // }

  if (world.visualizer) {
    VisualsDebugWindow visuals_debug;
    world.visualizer->visualizer.add_gui(
        "Visuals Debug", [&]() { visuals_debug.render(world); });
  }
  //
  while (!WindowShouldClose()) {
    world.step();
  }

  // De-initialize
  CloseWindow();
}
