
#include "physics/math/math.hpp" 
#include "physics/constraints/RevoluteJointConstraint.hpp"

// Import the sine function from the standard library
#include <math.h>
#include <iostream>
#include "Visualizer.hpp"
#include <raylib.h>
#include "physics/World.hpp"
#include "Interface.hpp"
#include <memory>


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



    
    // Body ball_6 = Body({0.0, 8.0, 0.0}, ori, lin_vel, ang_vel,  0.3*mass, ( 0.3*mass/12) * inertia_tensor, DYNAMIC);
    // Body ball_7 = Body({0.0, 9.0, 0.0}, ori, lin_vel, ang_vel, mass, ( 1.0/12) * inertia_tensor, DYNAMIC);
    // //Body ball_8 = Body({0.0, 10.0, 0.0}, ori, lin_vel, ang_vel, 4.0 * mass,  ( 1.0/12) *  inertia_tensor, DYNAMIC);
    // //Body ball_9 = Body({0.0, 11.0, 0.0}, ori, lin_vel, ang_vel, 0.1 * mass, ( 00.1/12) * inertia_tensor, DYNAMIC);
    // //Body ball_10 = Body({0.0, 12.0, 0.1}, ori, lin_vel, ang_vel, 1*mass, ( 10.0/12) * inertia_tensor, DYNAMIC);
    
    
    
    uint base_id = world.create_body({0.0, 2.0, 0.0}, ori, lin_vel, ang_vel, mass, inertia_tensor, STATIC);
    world.set_body_capsule_collider(base_id, 0.2, 0.1);

    uint ball_1_id = world.create_body({0.0, 5.0, 0.0}, ori, lin_vel, ang_vel, .7 * mass, ( .7 * mass/12) * inertia_tensor, DYNAMIC);
    world.set_body_box_collider(ball_1_id, vec3(0.2, 1.0, 0.2));
    
    uint ball_2_id = world.create_body({0.0, 3.0, 0.0}, ori, lin_vel, ang_vel, mass,  ( 1.0/12) *  inertia_tensor, DYNAMIC);
    world.set_body_box_collider(ball_2_id, vec3(0.2, 2.0, 0.2));

    uint ball_3_id = world.create_body({0.0, 6.0, 0.0}, ori, lin_vel, ang_vel, .6* mass, ( .6* mass/12) * inertia_tensor, DYNAMIC);
    world.set_body_box_collider(ball_3_id, vec3(0.2, 1.5, 0.2));

    uint ball_4_id = world.create_body({0.0, 6.5, 0.0}, ori, lin_vel, ang_vel,  0.5* mass,  (0.5* mass/12) *  inertia_tensor, DYNAMIC);
    world.set_body_box_collider(ball_4_id, vec3(0.2, 1.5, 0.2));

    uint ball_5_id = world.create_body({0.0, 7.0, 0.0}, ori, lin_vel, ang_vel,  0.2*mass,  ( 0.2*mass/12) *  inertia_tensor, DYNAMIC);
    world.set_body_sphere_collider(ball_5_id, 0.2);
    
    uint ball_6_id = world.create_body({0.0, 8.0, 0.0}, ori, lin_vel, ang_vel,  0.3*mass, ( 0.3*mass/12) * inertia_tensor, DYNAMIC);
    world.set_body_sphere_collider(ball_6_id, 0.2);

    uint ball_7_id = world.create_body({0.0, 9.0, 0.0}, ori, lin_vel, ang_vel, mass, ( 1.0/12) * inertia_tensor, DYNAMIC);
    world.set_body_sphere_collider(ball_7_id, 0.2);

    uint ball_8_id = world.create_body({0.0, 10.0, 0.0}, ori, lin_vel, ang_vel, 4.0 * mass,  ( 1.0/12) *  inertia_tensor, DYNAMIC);
    world.set_body_sphere_collider(ball_8_id, 0.2);

    uint ball_9_id = world.create_body({0.0, 11.0, 0.0}, ori, lin_vel, ang_vel, 0.1 * mass, ( 00.1/12) * inertia_tensor, DYNAMIC);
    world.set_body_sphere_collider(ball_9_id, 0.2);

    uint ball_10_id = world.create_body({0.0, 12.0, 0.1}, ori, lin_vel, ang_vel, 1*mass, ( 10.0/12) * inertia_tensor, DYNAMIC);
    world.set_body_sphere_collider(ball_10_id, 0.2);



    //vec3 axis = {0.0, 0.0, 1.0};
    int joint_0 = world.create_revolute_constraint(base_id, ball_1_id, {0.0, 0.0, 1.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 0.0}, 1e-9,  1e9,  DRIVEN_BY_POSITION, false, -0.8, 0.8);
    int joint_1 = world.create_revolute_constraint(ball_1_id, ball_2_id, {1.0, 1, 1.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 0.0},  1e-12,  1e-9,  DRIVEN_BY_POSITION, false, -0.5, 0.5);
    int joint_2 = world.create_revolute_constraint(ball_2_id, ball_3_id, {0.0, 0.1, 1.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 0.0}, 1e-5,  1e-4,  FREE, false, -0.0, 0.0);
    int joint_3 = world.create_revolute_constraint(ball_3_id, ball_4_id, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 0.0}, 1e-6,  1e-4,  DRIVEN_BY_POSITION, false, -0.0, 0.0);
    int joint_4 = world.create_revolute_constraint(ball_4_id, ball_5_id, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 0.0}, 1e-4,  1e-4,  FREE, false, -0.0, 0.0);
    // world.create_revolute_constraint(ball_5_id, ball_6_id, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 0.0}, 1e-4, 1e-4,  FREE, false, -0.0, 0.0);
    // //world.create_revolute_constraint(ball_6_id, ball_7_id, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 0.0}, 0.0,  1e4,  FREE, false, -0.0, 0.0);
    // world.create_revolute_constraint(ball_7_id, ball_8_id, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 0.0}, 0.0,  1e4,  FREE, false, -0.0, 0.0);
    // world.create_revolute_constraint(ball_8_id, ball_9_id, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 0.0}, 0.0,  1e4,  FREE, false, -0.0, 0.0);
    // world.create_revolute_constraint(ball_9_id, ball_10_id, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 0.0}, 0, 1e4,  FREE, false, -0.0, 0.0);
    
    world.set_revolute_joint_target_angle(joint_0, PI/4);
    world.set_revolute_joint_target_angle(joint_1, -PI/6);
    world.set_revolute_joint_target_angle(joint_3, PI/2);
    
    world.set_gravity({0.0, -9.8, 0.0});

   
    
    Visualizer visualizer(1208, 720, "RoboVis");
    visualizer.set_up_camera();

    std::shared_ptr<robosim::World> world_ptr (&world);
    std::shared_ptr<Visualizer> vis_ptr (&visualizer);
    Interface interface = Interface(world_ptr, vis_ptr);

    scalar time_step = world.get_time_step();
    scalar prev_angle = 0;
    double t_o = GetTime();
    double target_angle = PI/6;
    double dt = PI/180;
    double sing = -1;
    while (!WindowShouldClose()) {
        
        auto info = world.get_revolute_joint_info(joint_0);
        prev_angle = info.current_angle;
        
        if ( (GetTime() - t_o) >  0.01) {
            target_angle += sing * dt;
            if (abs(target_angle) > PI/4){
                sing *= -1;
            }
            world.set_revolute_joint_target_angle(joint_1, target_angle);
            t_o = GetTime();
        }

        // Update the visualizer
        visualizer.update();
        world.step();
        interface.update();
        

        
    }

    // De-initialize
    CloseWindow();

    return 0;
    

}