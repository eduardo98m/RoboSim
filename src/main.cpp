
#include "physics/math/math.hpp" 
#include "physics/constraints/RevoluteJointConstraint.hpp"

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

    vec3 it_1 = inertia_tensor[0];
    std::cout << "Test " << it_1 << "\n";

    //std::cout << "holar" << body.get_rotational_generalized_inverse_mass(v) << "\n";

    robosim::World world = robosim::World(0.01, 20); 

    quat ori = quat({ 0.0, 0.0, 0.0, });//ti::quat_from_axis_angle({0.0, 0.0, 1.0}, 0.0);
    std::cout << "ori" << ori << "\n"; 

    Body base = Body({0.0f, 2.0f, 0.0f}, ori, lin_vel, ang_vel, mass, inertia_tensor, STATIC);

    Body ball_1 = Body({0.0, 3.0, 0.0}, ori, lin_vel, ang_vel, 4.0 * mass,  ( 1.0/12) *  inertia_tensor, DYNAMIC);
    Body ball_2 = Body({0.0, 5.0, 0.0}, ori, lin_vel, ang_vel, 3.0 * mass, ( 1.0/12) * inertia_tensor, DYNAMIC);
    Body ball_3 = Body({0.0, 6.0, 0.0}, ori, lin_vel, ang_vel, mass, ( 1.0/12) * inertia_tensor, DYNAMIC);

    Body ball_4 = Body({0.0, 6.5, 0.0}, ori, lin_vel, ang_vel, 4.0 * mass,  ( 1.0/12) *  inertia_tensor, DYNAMIC);
    Body ball_5 = Body({0.0, 7.0, 0.0}, ori, lin_vel, ang_vel, 4.0 * mass,  ( 1.0/12) *  inertia_tensor, DYNAMIC);
    Body ball_6 = Body({0.0, 8.0, 0.0}, ori, lin_vel, ang_vel, 3.0 * mass, ( 1.0/12) * inertia_tensor, DYNAMIC);
    Body ball_7 = Body({0.0, 9.0, 0.0}, ori, lin_vel, ang_vel, mass, ( 1.0/12) * inertia_tensor, DYNAMIC);
    Body ball_8 = Body({0.0, 10.0, 0.0}, ori, lin_vel, ang_vel, 4.0 * mass,  ( 1.0/12) *  inertia_tensor, DYNAMIC);
    Body ball_9 = Body({0.0, 11.0, 0.0}, ori, lin_vel, ang_vel, 0.1 * mass, ( 00.1/12) * inertia_tensor, DYNAMIC);
    Body ball_10 = Body({0.0, 12.0, 0.1}, ori, lin_vel, ang_vel, 1*mass, ( 10.0/12) * inertia_tensor, DYNAMIC);
    
    
    
    uint ball_1_id = world.add_body(ball_1);
    uint ball_2_id = world.add_body(ball_2);
    uint ball_3_id = world.add_body(ball_3);
    
    uint ball_4_id = world.add_body(ball_4);
    uint ball_5_id = world.add_body(ball_5);
    uint ball_6_id = world.add_body(ball_6);

    uint ball_7_id = world.add_body(ball_7);
    uint ball_8_id = world.add_body(ball_8);
    uint ball_9_id = world.add_body(ball_9);
    uint ball_10_id = world.add_body(ball_10);


    
    uint base_id = world.add_body(base);

    vec3 axis = {0.0, 0.0, 1.0};
    world.create_revolute_constraint(base_id, ball_1_id, {0.5, 0.0, 1.0} ,{0.0, 1.0, 0.0}, {0.0, 0.0, 0.0}, 0,    10, FREE, false, -0.8, 0.8);
    world.create_revolute_constraint(ball_1_id, ball_2_id, {1.0, 0.0, 1.0}, {0.0, 2.0, 0.0}, {0.0, 0.0, 0.0}, 0,  10,  FREE, false, -0.5, 0.5);
    world.create_revolute_constraint(ball_2_id, ball_3_id, {1.0, 0.0, 1.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 0.0}, 0,  10,  FREE, false, -0.0, 0.0);

    world.create_revolute_constraint(ball_3_id, ball_4_id, {1.0, 0.0, 1.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 0.0}, 0,  10,  FREE, false, -0.0, 0.0);
    world.create_revolute_constraint(ball_4_id, ball_5_id, {1.0, 0.0, 1.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 0.0}, 0,  10,  FREE, false, -0.0, 0.0);
    world.create_revolute_constraint(ball_5_id, ball_6_id, {1.0, 0.0, 1.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 0.0}, 0,  10,  FREE, false, -0.0, 0.0);
    world.create_revolute_constraint(ball_6_id, ball_7_id, {1.0, 0.0, 1.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 0.0}, 0,  10,  FREE, false, -0.0, 0.0);
    world.create_revolute_constraint(ball_7_id, ball_8_id, {1.0, 0.0, 1.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 0.0}, 0,  10,  FREE, false, -0.0, 0.0);
    world.create_revolute_constraint(ball_8_id, ball_9_id, {1.0, 0.0, 1.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 0.0}, 0,  10,  FREE, false, -0.0, 0.0);
    world.create_revolute_constraint(ball_9_id, ball_10_id, {1.0, 0.0, 1.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 0.0}, 0, 10,  FREE, false, -0.0, 0.0);
    
    
    world.set_gravity({0.0, -9.8, 0.0});

    
    Visualizer visualizer(1208, 720, "RoboVis");

    uint sphere_1_id = visualizer.add_box(vec3ToVector3(world.get_body_position(ball_1_id)), 
                                             quatToQuaternion(world.get_body_orientation(ball_1_id)), BLUE, 0.2f, 0.1f, 4.0f);
    
    uint sphere_2_id = visualizer.add_box(vec3ToVector3(world.get_body_position(ball_2_id)), 
                                                quatToQuaternion(world.get_body_orientation(ball_2_id)), RED, 0.2f, 2.0f, 0.2f);
    
    uint sphere_3_id = visualizer.add_box(vec3ToVector3(world.get_body_position(ball_3_id)), 
                                             quatToQuaternion(world.get_body_orientation(ball_3_id)), YELLOW, 0.2f, 1.5f, 0.2f);
    
    uint sphere_4_id = visualizer.add_sphere(vec3ToVector3(world.get_body_position(ball_3_id)), 
                                             quatToQuaternion(world.get_body_orientation(ball_3_id)), RED, 0.2f);
    
    uint sphere_5_id = visualizer.add_sphere(vec3ToVector3(world.get_body_position(ball_3_id)), 
                                             quatToQuaternion(world.get_body_orientation(ball_3_id)), YELLOW, 0.2f);

    uint sphere_6_id = visualizer.add_sphere(vec3ToVector3(world.get_body_position(ball_3_id)), 
                                             quatToQuaternion(world.get_body_orientation(ball_3_id)), RED, 0.2f);

    uint sphere_7_id = visualizer.add_sphere(vec3ToVector3(world.get_body_position(ball_3_id)), 
                                             quatToQuaternion(world.get_body_orientation(ball_3_id)), YELLOW, 0.2f);
    
     uint sphere_8_id = visualizer.add_sphere(vec3ToVector3(world.get_body_position(ball_3_id)), 
                                             quatToQuaternion(world.get_body_orientation(ball_3_id)), RED, 0.2f);

    uint sphere_9_id = visualizer.add_sphere(vec3ToVector3(world.get_body_position(ball_3_id)), 
                                             quatToQuaternion(world.get_body_orientation(ball_3_id)), YELLOW, 0.2f);

    uint sphere_10_id = visualizer.add_sphere(vec3ToVector3(world.get_body_position(ball_3_id)), 
                                             quatToQuaternion(world.get_body_orientation(ball_3_id)), GREEN, 0.2f);

                                    

    uint cyllinder_id = visualizer.add_cylinder({0.0f, 2.0f, 0.0f}, QuaternionIdentity(), GREEN, 0.2f, 0.1f);
    //uint plane = visualizer.add_plane({0.0f, -1.0f, 0.0f}, QuaternionIdentity(), WHITE, 10.0f, 10.0f); 

    // Set up the camera
    visualizer.set_up_camera();
    // Main game loop

    
    //world.step();
    //world.step();
    while (!WindowShouldClose()) {
        
        

        // Update the visualizer
        visualizer.update();
        world.step();

        //std::cout << "pos = " << world.get_body_position(ball_2_id) << std::endl;
        visualizer.update_visual_object_position_orientation(sphere_1_id, 
                                                            vec3ToVector3(world.get_body_position(ball_1_id)), 
                                                            quatToQuaternion(world.get_body_orientation(ball_1_id))
                                                            );
        visualizer.update_visual_object_position_orientation(sphere_2_id, 
                                                            vec3ToVector3(world.get_body_position(ball_2_id)), 
                                                            quatToQuaternion(world.get_body_orientation(ball_2_id))
                                                            );
        visualizer.update_visual_object_position_orientation(sphere_3_id, 
                                                            vec3ToVector3(world.get_body_position(ball_3_id)), 
                                                            quatToQuaternion(world.get_body_orientation(ball_3_id))
                                                            );
        
        visualizer.update_visual_object_position_orientation(sphere_4_id, 
                                                            vec3ToVector3(world.get_body_position(ball_4_id)), 
                                                            quatToQuaternion(world.get_body_orientation(ball_4_id))
                                                            );
        visualizer.update_visual_object_position_orientation(sphere_5_id, 
                                                            vec3ToVector3(world.get_body_position(ball_5_id)), 
                                                            quatToQuaternion(world.get_body_orientation(ball_5_id))
                                                            );
        visualizer.update_visual_object_position_orientation(sphere_6_id, 
                                                            vec3ToVector3(world.get_body_position(ball_6_id)), 
                                                            quatToQuaternion(world.get_body_orientation(ball_6_id))
                                                            );

        visualizer.update_visual_object_position_orientation(sphere_7_id, 
                                                            vec3ToVector3(world.get_body_position(ball_7_id)), 
                                                            quatToQuaternion(world.get_body_orientation(ball_7_id))
                                                            );
        visualizer.update_visual_object_position_orientation(sphere_8_id, 
                                                            vec3ToVector3(world.get_body_position(ball_8_id)), 
                                                            quatToQuaternion(world.get_body_orientation(ball_8_id))
                                                            );
        visualizer.update_visual_object_position_orientation(sphere_9_id, 
                                                            vec3ToVector3(world.get_body_position(ball_9_id)), 
                                                            quatToQuaternion(world.get_body_orientation(ball_9_id))
                                                            );

        visualizer.update_visual_object_position_orientation(sphere_10_id, 
                                                            vec3ToVector3(world.get_body_position(ball_10_id)), 
                                                            quatToQuaternion(world.get_body_orientation(ball_10_id))
                                                            );


        
    }

    // De-initialize
    CloseWindow();

    return 0;
    

}