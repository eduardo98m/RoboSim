
#include "physics/math/math.hpp"
#include "physicsAcc/API/World.hpp"
#include "VisualizerNew.hpp"

int main(int argc, char *argv[])
{
    rbvs::Visualizer visualizer = rbvs::Visualizer(1208, 720, "RoboVis");

    World world;

    world.create_body(BodyParams());


    while (!WindowShouldClose())
    {
        
        world.step();
        // Update the visualizer
        visualizer.update();

        // visualizer.update_visual_object_position_orientation(cube_id, new_pos, new_orientation);
    }

    // De-initialize
    CloseWindow();

}