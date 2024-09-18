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
#include <functional>

robosim::World prismatic_joint_test_scenario()
{

    robosim::World world = robosim::World(0.013, 20);
    quat ori = ti::quat_from_axis_angle({0.0, 0.0, 1.0}, 0.0);
    scalar mass = 0.05;
    mat3 inertia_tensor = mat3{1.0, 0.0, 0.0,
                               0.0, 1.0, 0.0,
                               0.0, 0.0, 1.0};

    vec3 ang_vel = vec3{0.0, 0.0, 0.0};
    vec3 lin_vel = {0.0, 0.0, 0.0};

    size_t base_id = world.create_body({0.0, 2.0, 0.0}, ori, lin_vel, ang_vel, mass, inertia_tensor, STATIC);
    world.attach_box_collider(base_id, vec3(0.1, 0.5, 0.1));
    world.attach_box_visual_shape(base_id, vec3(0.1, 0.5, 0.1));

    size_t link_1_id = world.create_body({0.0, 5.0, 0.0}, ori, {1.0, 0.0, 0.0}, ang_vel, 12.0 * mass, (.7 * mass / 12) * inertia_tensor, DYNAMIC);
    world.attach_box_collider(link_1_id, vec3(0.1, 0.3, 0.1));
    world.attach_box_visual_shape(link_1_id, vec3(0.1, 0.3, 0.1));

    size_t link_2_id = world.create_body({0.0, 3.0, 0.0}, ori, lin_vel, ang_vel, mass, (1.0 / 12) * inertia_tensor, DYNAMIC);
    world.attach_cylinder_collider(link_2_id, 0.1,  0.4);
    world.attach_cylinder_visual_shape(link_2_id, 0.1,  0.4);


    size_t ball_1_id = world.create_body({0.0, 6.0, 0.2}, ori, lin_vel, ang_vel, mass, (1.0 / 12) * inertia_tensor, DYNAMIC);
    world.attach_sphere_collider(ball_1_id, 0.1);
    world.attach_sphere_visual_shape(ball_1_id, 0.1);


    size_t ball_2_id = world.create_body({0.0, 5.0, -0.2}, ori, lin_vel, ang_vel, mass, (1.0 / 12) * inertia_tensor, DYNAMIC);
    world.attach_sphere_collider(ball_2_id, 0.1);
    world.attach_sphere_visual_shape(ball_2_id, 0.1);



    world.set_gravity({0.0, -9.8, 0.0});
    

    size_t joint_0 = world.create_prismatic_joint_constraint(base_id, link_2_id, {0.0, 1.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 0.0}, 0.1, 220.1, FREE, true, -.2, .2);
    size_t joint_1 = world.create_prismatic_joint_constraint(link_2_id, link_1_id, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 0.0}, 1e-4, 0.0, DRIVEN_BY_POSITION, true, -2.2, 2.2);
    
    world.set_prismatic_joint_target_position(joint_1, -1.0);
    return world;
}
