#pragma once

#include "scenarios/scenario.hpp"

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
#include <random>


robosim::World robot_arm_scenario(){


    robosim::World world = robosim::World(0.01, 20);
    quat ori = ti::quat_from_axis_angle({0.0, 0.0, 1.0}, 2.0);
    scalar mass = 1.0;
    mat3 inertia_tensor = mat3{1.0, 0.0, 0.0,
                               0.0, 1.0, 0.0,
                               0.0, 0.0, 1.0};
    
    vec3 ang_vel = vec3{0.0, 0.0, 0.0};
    vec3 lin_vel = {0.0, 0.0, 0.0};

    uint base_id = world.create_body({0.0, 2.0, 0.0}, ori, lin_vel, ang_vel, mass, inertia_tensor, STATIC);
    world.attach_box_collider(base_id, vec3(0.1, 0.5, 0.1));

    uint ball_1_id = world.create_body({0.0, 5.0, 0.0}, ori, lin_vel, ang_vel, 12.0 * mass, ( .7 * mass/12) * inertia_tensor, DYNAMIC);
    world.attach_box_collider(ball_1_id, vec3(0.1, 0.5, 0.1));

    uint ball_2_id = world.create_body({0.0, 3.0, 0.0}, ori, lin_vel, ang_vel, mass,  ( 1.0/12) *  inertia_tensor, DYNAMIC);
    world.attach_box_collider(ball_2_id, vec3(0.1, 0.3, 0.1));

    uint ball_3_id = world.create_body({0.0, 6.0, 0.0}, ori, lin_vel, ang_vel, .6* mass, ( .6* mass/12) * inertia_tensor, DYNAMIC);
    world.attach_box_collider(ball_3_id, vec3(0.1, 0.3, 0.1));

    uint ball_4_id = world.create_body({0.0, 6.5, 0.0}, ori, lin_vel, ang_vel,  0.5* mass,  (0.5* mass/12) *  inertia_tensor, DYNAMIC);
    world.attach_box_collider(ball_4_id, vec3(0.1, 0.75, 0.1));

    uint ball_5_id = world.create_body({0.0, 7.0, 0.0}, ori, lin_vel, ang_vel,  0.2*mass,  ( 0.2*mass/12) *  inertia_tensor, DYNAMIC);
    world.attach_sphere_collider(ball_5_id, 0.2);

    uint ball_6_id = world.create_body({0.0, 8.0, 0.0}, ori, lin_vel, ang_vel,  0.3*mass, ( 0.3*mass/12) * inertia_tensor, DYNAMIC);
    world.attach_sphere_collider(ball_6_id, 0.2);

    uint ball_7_id = world.create_body({0.0, 9.0, 0.0}, ori, lin_vel, ang_vel, mass, ( 1.0/12) * inertia_tensor, DYNAMIC);
    world.attach_sphere_collider(ball_7_id, 0.2);

    uint ball_8_id = world.create_body({0.0, 10.0, 0.0}, ori, lin_vel, ang_vel, 4.0 * mass,  ( 1.0/12) *  inertia_tensor, DYNAMIC);
    world.attach_sphere_collider(ball_8_id, 0.2);

    uint ball_9_id = world.create_body({0.0, 11.0, 0.0}, ori, lin_vel, ang_vel, 0.1 * mass, ( 00.1/12) * inertia_tensor, DYNAMIC);
    world.attach_sphere_collider(ball_9_id, 0.2);

    uint ball_10_id = world.create_body({0.0, 12.0, 0.1}, ori, lin_vel, ang_vel, 1*mass, ( 10.0/12) * inertia_tensor, DYNAMIC);
    world.attach_sphere_collider(ball_10_id, 0.2);

    
    world.set_gravity({0.0, -9.8, 0.0});
    

    int joint_0 = world.create_revolute_constraint(base_id, ball_1_id, {0.0, 0.0, 1.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 0.0}, 0.0,  150,  DRIVEN_BY_POSITION, false, -0.8, 0.8);
    int joint_1 = world.create_revolute_constraint(ball_1_id, ball_2_id, {1.0, 1.0, 1.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 0.0},  1e-9,  150,  DRIVEN_BY_POSITION, false, -0.5, 0.5);
    int joint_2 = world.create_revolute_constraint(ball_2_id, ball_3_id, {0.0, 0.1, 1.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 0.0}, 1e-4,  150,  FREE, false, -0.0, 0.0);
    int joint_3 = world.create_revolute_constraint(ball_3_id, ball_4_id, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 0.0}, 1e-8,  150,  DRIVEN_BY_POSITION, false, -0.0, 0.0);
    int joint_4 = world.create_revolute_constraint(ball_4_id, ball_5_id, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 0.0}, 1e-4,  150,  FREE, false, -0.0, 0.0);


    world.set_revolute_joint_target_angle(joint_0, PI/4);
    world.set_revolute_joint_target_angle(joint_1, -PI/6);
    world.set_revolute_joint_target_angle(joint_3, PI/2);

    return world;

}