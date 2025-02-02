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
#include <cfloat>  // Para FLT_MAX
#include "implot.h"

robosim::World box_sliding_scenario()
{

    robosim::World world = robosim::World(0.01, 20);
    quat ori = ti::quat_from_axis_angle({0.0, 0.0, 1.0}, 2.0);
    scalar mass = 1.0;
    mat3 inertia_tensor = mat3{1.0, 0.0, 0.0,
                               0.0, 1.0, 0.0,
                               0.0, 0.0, 1.0};

    vec3 ang_vel = vec3{0.0, 0.0, 0.0};

    quat orientation = ti::quat_from_axis_angle({0.0, 0.0, 1.0}, PI/6);
    // size_t plane_id = world.add_plane({0.0, 1.0, 0}, 0.0);
    size_t plane_id = world.create_body({0.0, 0.0, 0.0}, 
                                        orientation, 
                                        vec3{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, 
                                        500 * mass, 500 * (10.0 / 12) * inertia_tensor, 
                                        STATIC);
    world.attach_box_collider(plane_id, {50.0, 0.2, 50.0});
    world.attach_box_visual_shape(plane_id, {50.0, 0.2, 50.0});



    scalar side_lenght = 0.3;
    quat ori_zero = ti::quat_from_axis_angle({1.0, 0.0, 0.0}, 0.0);
    vec3 pos_zero = vec3{0.0, 0.0, 0.0};


    size_t box_id = world.create_body(ti::rotate(orientation, vec3{0.0, side_lenght + 0.2 + 0.01, 0.0}) , 
                                        orientation, 
                                        vec3{0.0, 0.0, 0.0}, 
                                        vec3{0.0, 0.0, 0.0}, 
                                        mass, 
                                        inertia_tensor, 
                                        DYNAMIC);

    world.attach_box_collider(box_id, 
                              {side_lenght, side_lenght, side_lenght},
                              pos_zero,
                              ori_zero,
                              0.5,
                              0.1, 0.1);
    
    world.attach_box_visual_shape(box_id, 
                                 {side_lenght, side_lenght, side_lenght}, 
                                 pos_zero,
                                 ori_zero,
                                rs::Color{
                                    .r = 20,
                                    .g = 255,
                                    .b = 20,
                                    .a = 255
                                }
                                 );



    // world.attach_sphere_collider(ball_id, 1.0);


    world.set_gravity({0.0, -9.8, 0.0});

    return world;
}

//------------------------------------------------------------------------------
// This function returns a pair of callbacks:
// 1. A simulation step callback that computes and stores the box position and energy data.
// 2. A GUI callback that displays two plots: one for Y position and one for energy vs. time.
std::pair<
    std::function<void(std::shared_ptr<robosim::World>, std::shared_ptr<Visualizer>)>,
    std::function<void()>>
box_sliding_functions(std::shared_ptr<robosim::World> world_ptr)
{
    // We assume that the box's id is 1 (as created in box_sliding_scenario).
    const size_t box_id = 1;
    // Box mass (assumed to be 1.0 for this scenario).
    const float box_mass = 1.0f;
    // Gravitational acceleration (in m/s^2).
    const float gravity = 9.8f;

    // Shared containers to store simulation data for plotting.
    auto times = std::make_shared<std::vector<float>>();
    auto y_positions = std::make_shared<std::vector<float>>();
    auto kinetic_energies = std::make_shared<std::vector<float>>();
    auto potential_energies = std::make_shared<std::vector<float>>();
    auto total_energies = std::make_shared<std::vector<float>>();

    // Variable to accumulate simulated time.
    auto sim_time = std::make_shared<float>(0.0f);

    // Callback executed on each simulation step.
    auto step_callback = [=](std::shared_ptr<robosim::World> world_ptr,
                             std::shared_ptr<Visualizer> vis_ptr)
    {
        // Assume the dt is constant (0.01 s) or obtain it from world_ptr->get_dt().
        const float dt = 0.01f;
        *sim_time += dt;
        times->push_back(*sim_time);

        // Get the current position of the box.
        // We assume that get_body_position returns a vec3.
        vec3 pos = world_ptr->get_body_position(box_id);
        y_positions->push_back(pos.y);

        // Get the current velocity of the box.
        // We assume that get_body_velocity returns a vec3.
        vec3 vel = world_ptr->get_body_velocity(box_id);
        float speed_sq = vel.x * vel.x + vel.y * vel.y + vel.z * vel.z;

        // Compute kinetic energy: (1/2) * mass * speed^2.
        float kinetic = 0.5f * box_mass * speed_sq;
        // Compute potential energy: mass * g * height.
        float potential = box_mass * gravity * pos.y;
        // Total mechanical energy.
        float total = kinetic + potential;

        kinetic_energies->push_back(kinetic);
        potential_energies->push_back(potential);
        total_energies->push_back(total);
    };

    // GUI callback to display simulation data and two flexible plots using ImPlot.
    auto gui_interface = [=]()
    {
        ImGui::Begin("Simulation Data");
        ImPlot::CreateContext();
        // Display current simulation time and current Y position.
        ImGui::Text("Current time: %.3f s", *sim_time);
        if (!y_positions->empty())
        {
            ImGui::Text("Current Y position: %.3f", y_positions->back());
        }

        // Get the available region size for the plots.
        ImVec2 avail_size = ImGui::GetContentRegionAvail();
        // Split the available vertical space between two plots.
        ImVec2 plot_size = ImVec2(avail_size.x, avail_size.y * 0.45f);

        //---------- First Plot: Y Position vs. Time ----------
        if (ImPlot::BeginPlot("Y Position vs Time", plot_size))
        {
            ImPlot::SetupAxes("Time (s)", "Y Position");
            ImPlot::PlotLine("Y", times->data(), y_positions->data(), static_cast<int>(y_positions->size()));
            ImPlot::EndPlot();
        }

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        //---------- Second Plot: Energies vs. Time ----------
        if (ImPlot::BeginPlot("Energy vs Time", plot_size))
        {
            ImPlot::SetupAxes("Time (s)", "Energy (J)");
            // Plot Kinetic Energy.
            ImPlot::PlotLine("Kinetic Energy", times->data(), kinetic_energies->data(), static_cast<int>(times->size()));
            // Plot Potential Energy.
            ImPlot::PlotLine("Potential Energy", times->data(), potential_energies->data(), static_cast<int>(times->size()));
            // Plot Total Energy.
            ImPlot::PlotLine("Total Energy", times->data(), total_energies->data(), static_cast<int>(times->size()));
            ImPlot::EndPlot();
        }

        ImGui::End();
    };

    return {step_callback, gui_interface};
}