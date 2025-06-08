
#include "physics/math/math.hpp"
#include "physicsAcc/API/World.hpp"
#include "physicsAcc/Utils/PrintUtils.hpp"
#include "physicsAcc/Utils/DebugWindows.hpp"

#include "VisualizerNew.hpp"

int main(int argc, char *argv[])
{
    rbvs::Visualizer visualizer = rbvs::Visualizer(1208, 720, "RoboVis");

    World world;

    size_t body_1 = world.create_body(BodyParams{
        .type = BodyType::STATIC,
        .orientation = ti::quat_from_axis_angle({1.0, 1.0, 1.0}, 0.75)
        });

    size_t body_2 = world.create_body(BodyParams({
        .mass = 1.0,    
        .inertia_tensor = mat3{1.0, 0.0, 0.0, 
            0.0, 1.0, 0.0, 
            0.0, 0.0, 500.0},
        .position = {-5.0, 0.0, 0.0}
        
    }));    


    size_t joint_a = world.create_revolute_joint(RevoluteJointParams{
        .body_1 = body_1,
        .body_2 = body_2,
        .aligned_axis = {0.0, 0.0, 1.0},
        .limit_axis = {0.0, 1.0, 0.0},
        .r_1 =  {-5.0, 0.0, 5.0},
        .actuation_type = JointActuationType::FREE,
    });


    // size_t joint_a = world.create_rotational_joint(PrismaticJointParams{
    //     .body_1 = body_1,
    //     .body_2 = body_2,
    //     .moving_axis = {1.0, 0.0, 0.0},
    //     .r_1 =  {-5.0, 0.0, 5.0},
    //     .actuation_type = JointActuationType::FREE,
    // });

    rbvs::Entity body_1_model = visualizer.create_model(rbvs::ModelParams{
        .position = {1.0, 0.0, 0.0},
        .color = RED,
        .model_type = rbvs::ModelType::BOX,
        
    });

    rbvs::Entity body_2_model = visualizer.create_model(rbvs::ModelParams{
        .position = {1.0, 0.0, 0.0},
        .color = BLUE,
        .model_type = rbvs::ModelType::BOX,
        
    });

    DebugGUIHandler debug_gui_handler;
    visualizer.add_gui("Physics Debugger", [&]() {
        debug_gui_handler.render_debug_uis(world);
    });

    while (!WindowShouldClose())
    {
        
        world.step();
        // Update the visualizer
        visualizer.update();

        visualizer.update_model(rbvs::ModelUpdateParams{
            .entity = body_1_model,
            .position = ti::to_raylib(world.bodies.position[body_1]),
            .orientation = ti::to_raylib(world.bodies.orientation[body_1])
        });

        visualizer.update_model(rbvs::ModelUpdateParams{
            .entity = body_2_model,
            .position = ti::to_raylib(world.bodies.position[body_2]),
            .orientation = ti::to_raylib(world.bodies.orientation[body_2])
        });

        
    }

    // De-initialize
    CloseWindow();

}