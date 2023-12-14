
#include "physics/math/math.hpp" 
#include "physics/bodies/DynamicBody.hpp"


// Import the sine function from the standard library
#include <math.h>
#include <iostream>
#include "Visualizer.hpp"
#include <raylib.h>
#include "physics/World.hpp"

int main(int argc, char *argv[]){



    // Define a vector 3
    vec3 v = vec3{1.0, 2.1, 69.0};

    // Create a dynamic body
    vec3 pos = vec3{0.6, 9.4, 2.0};
    quat ori = quat{1.0, 0.0, 0.0, 0.0};
    vec3 lin_vel = vec3{0.0, 0.0, 0.0};
    vec3 ang_vel = vec3{0.0, 0.0, 0.0};
    scalar mass = 1.0;
    mat3 inertia_tensor = mat3{1.0, 0.0, 0.0,
                               0.0, 1.0, 0.0,
                               0.0, 0.0, 1.0};
    Body body = Body(pos, ori);

    // Print the vector
    std::cout << "The vector is: " << v << std::endl;
    // Print the properties of the body
    std::cout << "The position of the body is: " << body.position << std::endl;

    std::cout << "inertia_tensor" << inertia_tensor << std::endl;

    std::cout << "holar" << body.get_rotational_generalized_inverse_mass(v) << "\n";

    robosim::World world = robosim::World(); 

    uint body_id = world.add_body(body);

    std::cout << "Getting the cube position = " << world.get_body_position(body_id) << "\n";
    std::cout << "Getting the cube orientation = " << world.get_body_orientation(body_id) << "\n";
    world.set_gravity({0.0, 0.0, -9.8});
    world.step();
    world.step();
    world.step();
    std::cout << "Getting the cube position = " << world.get_body_position(body_id) << "\n";
    std::cout << "Getting the cube orientation = " << world.get_body_orientation(body_id) << "\n";

    Visualizer visualizer(1208, 720, "RoboVis");

    // Set up the camera
    visualizer.set_up_camera();
    

    uint sphere_id = visualizer.add_sphere({0.0f, 2.0f, 0.0f}, QuaternionIdentity(), BLUE, 1.0f);
    uint cylinder_id = visualizer.add_cylinder({1.0f, 0.0f, 0.0f}, QuaternionIdentity(), GREEN, 1.0f, 2.0f);
    uint cone_id = visualizer.add_cone({-1.0f, 0.0f, 0.0f}, QuaternionIdentity(), YELLOW, 1.0f, 2.0f);
    uint plane = visualizer.add_plane({0.0f, -1.0f, 0.0f}, QuaternionIdentity(), WHITE, 10.0f, 10.0f); 

    // uint monkey_id = visualizer.add_mesh("../resources/models/monkey.obj", {0.0f, 1.0f, 1.0f}, QuaternionIdentity(), RED, 1.0f);

    //visualizer.load_shader("../resources/shaders/lighting.vs" , "../resources/shaders/lighting.fs");
    //visualizer.set_up_lighting();
    
    // Define the sphere
    Vector3 new_pos = {0.0f, 0.0f, 0.0f};
    Quaternion new_orientation = QuaternionIdentity();
    uint cube_id = visualizer.add_cube(new_pos, QuaternionIdentity(), RED, 1.0, 1.0, 1.0);
    SetTargetFPS(60);
    // Main game loop
    while (!WindowShouldClose()) {

        // Update the visualizer
        visualizer.update();

        float floatHeight = 2.0f;
        float rotationSpeed = 2.0f;

        new_pos.y = floatHeight * sinf(GetTime());
        new_orientation = QuaternionFromEuler(0.0f, GetTime() * rotationSpeed, 0.0f);

        visualizer.update_visual_object_position_orientation(cube_id, new_pos, new_orientation);

    }

    // De-initialize
    CloseWindow();

    return 0;
    

}