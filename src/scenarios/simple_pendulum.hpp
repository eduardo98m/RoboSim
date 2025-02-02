#pragma once
#include "scenarios/scenario.hpp"

// Import the sine function from the standard library
#include <math.h>
#include <iostream>
#include "Visualizer.hpp"
#include <raylib.h>
#include "physics/World.hpp"
#include "Interface.hpp"
#include <memory>
#include <random>
#include "implot.h"

robosim::World pendulum_scenario()
{

    robosim::World world = robosim::World(0.01, 20);
    quat ori = ti::quat_from_axis_angle({0.0, 0.0, 1.0}, 0.0);
    scalar mass = 1.0;
    mat3 inertia_tensor = mat3{1.0, 0.0, 0.0,
                               0.0, 1.0, 0.0,
                               0.0, 0.0, 1.0};

    //
    size_t anchor_id = world.create_body({0.0, 0.0, 0.0}, ori, vec3{-0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, 0.1, inertia_tensor, BodyType::STATIC);

    // We are going to set the pendulum at 30 cm from the center
    scalar pendulum_mass = 0.8;                                       // 0.8 kg
    mat3 pendulum_intertia = pendulum_mass * 1 / 12 * inertia_tensor; // Intertia tensor of a solid sphere
    size_t pendulum_id = world.create_body({-0.3, 0.0, 0.0}, ori, vec3{-0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, pendulum_mass, pendulum_intertia, BodyType::DYNAMIC);

    // Now we set the constraint

    world.create_revolute_constraint(
        anchor_id,
        pendulum_id,
        vec3{0.0, 0.0, 1.0},
        vec3{0.0, 0.0, 0.0},
        vec3{0.3, 0.0, 0.0},
        0.0,
        0.0);

    world.set_gravity({0.0, -9.8, 0.0});

    world.attach_sphere_visual_shape(pendulum_id, 0.03, {0.0, 0.0, 0.0}, ori, rs::Color{.r = 235, .g = 101, .b = 52});

    return world;
}

//------------------------------------------------------------------------------
// This function returns a pair of callbacks:
// 1. A simulation step callback that computes and stores the pendulum's angle (θ),
//    angular velocity, and energy data.
// 2. A GUI callback that displays two ImPlot plots: one for orientation & angular velocity,
//    and another for energy vs. time.
std::pair<
    std::function<void(std::shared_ptr<robosim::World>, std::shared_ptr<Visualizer>)>,
    std::function<void()>>
pendulum_functions(std::shared_ptr<robosim::World> world_ptr)
{
    // We assume that the pendulum's id is 1 (the anchor is created first).
    const size_t pendulum_id = 1;
    const float pendulum_mass = 0.8f;
    const float gravity = 9.8f;
    // Compute the moment of inertia about the Z-axis.
    // Using the provided formula: I = pendulum_mass / 12
    const float I = pendulum_mass / 12.0f;

    // Shared containers to store simulation data for plotting.
    auto times = std::make_shared<std::vector<float>>();
    auto thetas = std::make_shared<std::vector<float>>();
    auto angular_velocities = std::make_shared<std::vector<float>>();
    auto kinetic_energies = std::make_shared<std::vector<float>>();
    auto potential_energies = std::make_shared<std::vector<float>>();
    auto total_energies = std::make_shared<std::vector<float>>();

    // Variable to accumulate simulated time.
    auto sim_time = std::make_shared<float>(0.0f);

    // Simulation step callback.
    auto step_callback = [=](std::shared_ptr<robosim::World> world_ptr,
                             std::shared_ptr<Visualizer> vis_ptr)
    {
        // Assume a constant dt of 0.01 s.
        const float dt = 0.01f;
        *sim_time += dt;
        times->push_back(*sim_time);

        // --- Orientation and Angular Velocity ---

        // Get the current orientation of the pendulum.
        // We assume the pendulum rotates only about the Z-axis.
        quat q = world_ptr->get_body_orientation(pendulum_id);
        vec3 p = world_ptr->get_body_position(pendulum_id);
        //ti::eule
        // Extract theta using: theta = 2 * atan2(q.z, q.w)
        // (Adjust if your quaternion format differs.)
        float theta = atan2(p.y, p.x);
        thetas->push_back(theta);

        // Get the current angular velocity.
        // We assume get_body_angular_velocity returns a vec3,
        // and we take the Z component as the relevant angular velocity.
        vec3 ang_vel = world_ptr->get_body_angular_velocity(pendulum_id);
        float omega = ang_vel.z;
        angular_velocities->push_back(omega);

        // --- Energy Calculations ---

        // Rotational kinetic energy: (1/2) * I * ω^2.
        float kinetic_rot = 0.5f * I * omega * omega;

        vec3 lin_vel = world_ptr->get_body_velocity(pendulum_id);
        scalar vel = ti::magnitude(lin_vel);// ti::
        float kinetic_lin = 0.5f * vel * vel * pendulum_mass;
        float kinetic = kinetic_lin +  kinetic_rot;

        // Potential energy: m * g * (y position).
        // We use the pendulum's y-coordinate relative to the anchor at (0,0,0).
        vec3 pos = world_ptr->get_body_position(pendulum_id);
        float potential = pendulum_mass * gravity * pos.y;

        float total = kinetic + potential;

        kinetic_energies->push_back(kinetic);
        potential_energies->push_back(potential);
        total_energies->push_back(total);
    };

    // GUI callback for plotting data using ImPlot.
    const int max_points = 1000;

    auto gui_interface = [=]()
    {
        ImGui::Begin("Pendulum Simulation Data");
        ImPlot::CreateContext();

        // Display current simulation data
        ImGui::Text("Current time: %.3f s", *sim_time);
        if (!thetas->empty())
        {
            ImGui::Text("Current angle: %.3f rad", thetas->back());
        }
        if (!angular_velocities->empty())
        {
            ImGui::Text("Current angular velocity: %.3f rad/s", angular_velocities->back());
        }

        // Get available region size and allocate space for the plots.
        ImVec2 avail_size = ImGui::GetContentRegionAvail();
        ImVec2 plot_size = ImVec2(avail_size.x, avail_size.y * 0.45f);

        // Compute the starting index for the data to plot
        int count = static_cast<int>(times->size());
        int start_idx = (count > max_points) ? (count - max_points) : 0;
        int plot_count = count - start_idx;

        //---------- First Plot: Orientation (θ) and Angular Velocity ----------
        if (ImPlot::BeginPlot("Orientation & Angular Velocity vs Time", plot_size))
        {
            ImPlot::SetupAxes("Time (s)", "Value", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
            ImPlot::PlotLine("θ (rad)",
                             times->data() + start_idx,
                             thetas->data() + start_idx,
                             plot_count);
            ImPlot::PlotLine("Angular Velocity (rad/s)",
                             times->data() + start_idx,
                             angular_velocities->data() + start_idx,
                             plot_count);
            ImPlot::EndPlot();
        }

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        //---------- Second Plot: Energies vs Time ----------
        if (ImPlot::BeginPlot("Energy vs Time", plot_size))
        {
            ImPlot::SetupAxes("Time (s)", "Energy (J)", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
            ImPlot::PlotLine("Kinetic Energy",
                             times->data() + start_idx,
                             kinetic_energies->data() + start_idx,
                             plot_count);
            ImPlot::PlotLine("Potential Energy",
                             times->data() + start_idx,
                             potential_energies->data() + start_idx,
                             plot_count);
            ImPlot::PlotLine("Total Energy",
                             times->data() + start_idx,
                             total_energies->data() + start_idx,
                             plot_count);
            ImPlot::EndPlot();
        }

        ImGui::End();
    };

    return {step_callback, gui_interface};
}