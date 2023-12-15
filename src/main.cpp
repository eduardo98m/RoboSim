
#include "physics/math/math.hpp" 
#include "physics/constraints/PositionalConstraint.hpp"

// Import the sine function from the standard library
#include <math.h>
#include <iostream>
#include "Visualizer.hpp"
#include <raylib.h>
#include "physics/World.hpp"


Vector3 vec3ToVector3(vec3 v){
    return {(float)v.x, (float)v.y, (float)v.z};
    }  
Quaternion quatToQuaternion(quat q){
    return {(float)q.x, (float)q.y, (float)q.z, (float)q.w};
    } 

int main(int argc, char *argv[]){



    // Define a vector 3
    vec3 v = vec3{0.0, 0.0, 0.0};

    // Create a dynamic body
    vec3 pos = vec3{0.0, 0.0, 12.0};
    quat ori = quat{1.0, 0.0, 0.0, 0.0};
    vec3 lin_vel = vec3{0.0, 0.0, 0.0};
    vec3 ang_vel = vec3{0.0, 0.0, 0.0};
    scalar mass = 1.0;
    mat3 inertia_tensor = mat3{1.0, 0.0, 0.0,
                               0.0, 1.0, 0.0,
                               0.0, 0.0, 1.0};
    

    // Print the vector
    std::cout << "The vector is: " << v << std::endl;
    // Print the properties of the body

    std::cout << "inertia_tensor" << inertia_tensor << std::endl;

    //std::cout << "holar" << body.get_rotational_generalized_inverse_mass(v) << "\n";

    robosim::World world = robosim::World(); 


    Body ball_1 = Body({0.0, 2.0, 0.0}, ori, lin_vel, ang_vel, mass, inertia_tensor, DYNAMIC);
    
    Body ball_2 = Body({0.0, 2.0, 2.0}, ori, lin_vel, ang_vel, mass, inertia_tensor, DYNAMIC);
    Body base = Body({0.0f, 2.0f, 0.0f}, ori, lin_vel, ang_vel, mass, inertia_tensor, STATIC);

    uint ball_1_id = world.add_body(ball_1);
    uint ball_2_id = world.add_body(ball_2);
    uint base_id = world.add_body(base);

    world.create_positional_constraint(base_id, ball_1_id, {0.0, 1.0, 0.0}, {0.0, 0.0, 0.0}, 0,  0);
    world.create_positional_constraint(ball_1_id, ball_2_id, {0.0, 0.0, -2.0}, {0.0, 0.0, 0.0}, 0,  0);
    world.set_gravity({0.0, -90.8, 0.0});

    


    Visualizer visualizer(1208, 720, "RoboVis");

    uint sphere_2_id = visualizer.add_sphere(vec3ToVector3(world.get_body_position(ball_1_id)), 
                                             quatToQuaternion(world.get_body_orientation(ball_2_id)), BLUE, 0.2f);
    
    uint sphere_1_id = visualizer.add_sphere(vec3ToVector3(world.get_body_position(ball_2_id)), 
                                                quatToQuaternion(world.get_body_orientation(ball_2_id)), RED, 0.2f);

    uint cyllinder_id = visualizer.add_cylinder({0.0f, 2.0f, 0.0f}, QuaternionIdentity(), GREEN, 0.2f, 0.1f);
    //uint plane = visualizer.add_plane({0.0f, -1.0f, 0.0f}, QuaternionIdentity(), WHITE, 10.0f, 10.0f); 

    // Set up the camera
    visualizer.set_up_camera();
    // Main game loop
    while (!WindowShouldClose()) {
        
        world.step();

        // Update the visualizer
        visualizer.update();

        std::cout << "pos = " << world.get_body_position(ball_2_id) << std::endl;
        visualizer.update_visual_object_position_orientation(sphere_1_id, 
                                                            vec3ToVector3(world.get_body_position(ball_1_id)), 
                                                            quatToQuaternion(world.get_body_orientation(ball_1_id))
                                                            );
        visualizer.update_visual_object_position_orientation(sphere_2_id, 
                                                            vec3ToVector3(world.get_body_position(ball_2_id)), 
                                                            quatToQuaternion(world.get_body_orientation(ball_2_id))
                                                            );

    }

    // De-initialize
    CloseWindow();

    return 0;
    

}