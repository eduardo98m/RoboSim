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

robosim::World articulated_system_scenario()
{

    robosim::World world = robosim::World(0.01, 120);
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

    size_t link_1_id = world.create_body({0.0, 5.0, 0.0}, ori, lin_vel, ang_vel, 12.0 * mass, (.7 * mass / 12) * inertia_tensor, DYNAMIC);
    world.attach_box_collider(link_1_id, vec3(0.1, 0.3, 0.1));
    world.attach_box_visual_shape(link_1_id, vec3(0.1, 0.3, 0.1));

    size_t link_2_id = world.create_body({0.0, 3.0, 0.0}, ori, lin_vel, ang_vel, mass, (1.0 / 12) * inertia_tensor, DYNAMIC);
    world.attach_cylinder_collider(link_2_id, 0.1,  0.4);
    world.attach_cylinder_visual_shape(link_2_id, 0.1,  0.4);

    size_t link_3_id = world.create_body({0.0, 6.0, 0.0}, ori, lin_vel, ang_vel, .6 * mass, (.6 * mass / 12) * inertia_tensor, DYNAMIC);
    world.attach_box_collider(link_3_id, vec3(0.1, 0.3, 0.1));
    world.attach_box_visual_shape(link_3_id, vec3(0.1, 0.3, 0.1));

    size_t link_4_id = world.create_body({0.0, 6.5, 0.0}, ori, lin_vel, ang_vel, 0.5 * mass, (0.5 * mass / 12) * inertia_tensor, DYNAMIC);
    world.attach_box_collider(link_4_id, vec3(0.1, 0.5, 0.1));
    world.attach_box_visual_shape(link_4_id, vec3(0.1, 0.5, 0.1));

    size_t link_5_id = world.create_body({0.0, 7.0, 0.0}, ori, lin_vel, ang_vel, 0.2 * mass, (0.2 * mass / 12) * inertia_tensor, DYNAMIC);
    world.attach_sphere_collider(link_5_id, 0.2);
    world.attach_sphere_visual_shape(link_5_id, 0.2);

    size_t link_6_id = world.create_body({0.0, 8.0, 0.0}, ori, lin_vel, ang_vel, 0.3 * mass, (0.3 * mass / 12) * inertia_tensor, DYNAMIC);
    world.attach_sphere_collider(link_6_id, 0.2);
    world.attach_sphere_visual_shape(link_6_id, 0.2);

    size_t link_7_id = world.create_body({0.0, 9.0, 0.0}, ori, lin_vel, ang_vel, mass, (1.0 / 12) * inertia_tensor, DYNAMIC);
    world.attach_sphere_collider(link_7_id, 0.2);
    world.attach_sphere_visual_shape(link_7_id, 0.2);

    size_t link_8_id = world.create_body({0.0, 10.0, 0.0}, ori, lin_vel, ang_vel, 4.0 * mass, (1.0 / 12) * inertia_tensor, DYNAMIC);
    world.attach_sphere_collider(link_8_id, 0.2);
    world.attach_sphere_visual_shape(link_8_id, 0.2);

    size_t link_9_id = world.create_body({0.0, 11.0, 0.0}, ori, lin_vel, ang_vel, 0.1 * mass, (00.1 / 12) * inertia_tensor, DYNAMIC);
    world.attach_sphere_collider(link_9_id, 0.2);
    world.attach_sphere_visual_shape(link_9_id, 0.2);

    size_t link_10_id = world.create_body({0.0, 12.0, 0.1}, ori, lin_vel, ang_vel, 1 * mass, (10.0 / 12) * inertia_tensor, DYNAMIC);
    world.attach_sphere_collider(link_10_id, 0.2);
    world.attach_sphere_visual_shape(link_10_id, 0.2);

    int plane_id = world.create_body({0.0, 0.0, 0.0}, ti::quat_from_axis_angle({0.0, 1.0, 0.0}, 0.0), vec3{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, 500 * mass, 500 * (10.0 / 12) * inertia_tensor, STATIC);
    world.attach_box_collider(plane_id, {50.0, 0.2, 50.0});
    world.attach_box_visual_shape(plane_id, {50.0, 0.2, 50.0});

    world.set_gravity({0.0, -9.8, 0.0});
    

    size_t joint_0 = world.create_revolute_constraint(base_id, link_1_id, {0.0, 0.0, 1.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 0.0}, 0.0, 0.0, DRIVEN_BY_POSITION, true, -PI/2, PI/2);
    size_t joint_1 = world.create_revolute_constraint(link_1_id, link_2_id, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 0.0}, 0.0, 0.0, DRIVEN_BY_POSITION, true   , -0.5, 0.5);
    size_t joint_2 = world.create_revolute_constraint(link_2_id, link_3_id, {0.0, 0.0, 1.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 0.0}, 0.0, 0.0, DRIVEN_BY_POSITION, false, -0.0, 0.0);
    size_t joint_3 = world.create_prismatic_joint_constraint(link_3_id, link_4_id, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 0.0}, 0.0, 200.0, FREE, true, -0.2, 0.2);
    size_t joint_4 = world.create_revolute_constraint(link_4_id, link_5_id, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 0.0}, 0.0, 0.0, DRIVEN_BY_POSITION, false, -0.0, 0.0);
    size_t joint_5 = world.create_prismatic_joint_constraint(link_5_id, link_6_id, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 0.0}, 0.0, 0.0, DRIVEN_BY_POSITION, true, -PI, PI);

    size_t joint_6 = world.create_fixed_joint_constraint(link_3_id, link_10_id, {0.3, 0.0, 0.0}, {0.0, 0.0, 0.0});

    // Lets create the articulates systems:
    std::vector<size_t> joint_ids = {joint_0, joint_1, joint_2, joint_3, joint_4, joint_5};
    std::vector<JointType> joint_types = {JointType::HINGE, JointType::HINGE, JointType::HINGE, JointType::PRISMATIC, JointType::HINGE, JointType::PRISMATIC};
    std::vector<size_t> link_ids = {base_id, link_1_id, link_2_id, link_3_id, link_4_id, link_5_id, link_6_id};

    size_t articulated_system_id = world.create_articulated_system(joint_ids, joint_types, link_ids);

    return world;
}

std::pair<
    std::function<void(std::shared_ptr<robosim::World>, std::shared_ptr<Visualizer>)>,
    std::function<void()>>
create_functions(std::shared_ptr<robosim::World> world_ptr)
{
    std::vector<scalar> joint_values = world_ptr->get_articulated_system_state(0);

    static std::vector<float> target_values;
    target_values = {};
    for (int i = 7; i < joint_values.size(); i++)
    {
        target_values.push_back(0.0f);
    }

    auto gui_interface{
        [](void)
        {
            ImGui::Begin("Joint target angles");

            for (int i = 0; i < target_values.size(); i++)
            {
                ImGui::SliderFloat(("Joint " + std::to_string(i)).c_str(), &target_values[i], -180.0, 180.0, "%.2f");
            }
            ImGui::End();
        }};

    auto step_callback{
        [](std::shared_ptr<robosim::World> world_ptr, std::shared_ptr<Visualizer> vis_ptr)
        {
            std::vector<scalar> target_scalars(target_values.size());
            for (int i = 0; i < target_values.size(); i++)
            {
                target_scalars[i] =  target_values[i] * PI/180.0;
            }

            world_ptr->set_articulated_system_joint_targets(0, target_scalars);
        }};

    return {
        step_callback,
        gui_interface};
}