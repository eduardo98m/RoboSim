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

void ray_cast_callback(std::shared_ptr<robosim::World> world, std::shared_ptr<Visualizer> vis)
{

    // Define a static variable for tracking angle
    static scalar ray_angle = 0.0;
    static std::deque<vec3> stored_points;

    static bool first_exec = true;
    static std::shared_ptr<hpp::fcl::Capsule> ray_collider;
    static size_t id;

    // Increment ray angle and wrap around at 360 degrees
    ray_angle += 1.0f; // Adjust the increment as needed for desired spinning speed
    ray_angle = fmod(ray_angle, 360.0f);

    // Calculate the starting and ending points for the ray based on the current angle
    float radius = 10.0f; // Adjust this for desired ray length
    vec3 start = {0.0, 0.5, 0.0};
    vec3 end = {radius * ti::cos(ray_angle * PI / 180), 0.5, radius * ti::sin(ray_angle * PI / 180)};

    std::vector<vec3> points = world->raycast(start, end);

    for (const auto &p : points)
    {
        stored_points.push_back(p);
        // If the number of stored points exceeds 50, remove the oldest points
        if (stored_points.size() > 200)
        {
            stored_points.pop_front();
        }
    }

    // Draw the ray points
    for (auto const &p : stored_points)
    {
        vis->draw_sphere(ti::to_raylib(p), 0.05, RED);
    }

    vis->draw_line(ti::to_raylib(start), ti::to_raylib(end), RED);

}

robosim::World ray_cast_scenario()
{

    const float X_MIN = -2.5f, X_MAX = 2.5f;
    const float Y_MIN = 0.1f, Y_MAX = 5.1f;
    const float Z_MIN = -2.5f, Z_MAX = 2.5f;
    const float RADIUS = 0.4f;
    const int GRID_SIZE = 3;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> disX(X_MIN, X_MAX);
    std::uniform_real_distribution<float> disY(Y_MIN, Y_MAX);
    std::uniform_real_distribution<float> disZ(Z_MIN, Z_MAX);
    std::uniform_real_distribution<scalar> radius_dist(0.05, 0.4);

    // Generate positions for the balls in a 10x10x10 grid
    std::vector<vec3> ball_positions;
    float stepX = (X_MAX - X_MIN) / GRID_SIZE;
    float stepY = (Y_MAX - Y_MIN) / GRID_SIZE;
    float stepZ = (Z_MAX - Z_MIN) / GRID_SIZE;
    for (int i = 0; i < GRID_SIZE; ++i)
    {
        for (int j = 0; j < GRID_SIZE; ++j)
        {
            for (int k = 0; k < GRID_SIZE; ++k)
            {
                float x = X_MIN + i * stepX + stepX / 2;
                float y = Y_MIN + j * stepY + stepY / 2;
                float z = Z_MIN + k * stepZ + stepZ / 2;
                ball_positions.push_back({x, y, z});
            }
        }
    }

    robosim::World world = robosim::World(0.01, 20);
    quat ori = ti::quat_from_axis_angle({0.0, 0.0, 1.0}, 2.0);
    scalar mass = 1.0;
    mat3 inertia_tensor = mat3{1.0, 0.0, 0.0,
                               0.0, 1.0, 0.0,
                               0.0, 0.0, 1.0};

    vec3 ang_vel = vec3{0.0, 0.0, 0.0};

    mat3 cursed_cylinder_inertia = mat3{
        0.0700, 0.0000, 0.0000,
        0.0000, 0.0700, 0.0000,
        0.0000, 0.0000, 0.0800};

    // uint plane_id = world.add_plane({0.0, 1.0, 0}, 0.0);
    int plane_id = world.create_body({0.0, 0.0, 0.0}, ti::quat_from_axis_angle({0.0, 1.0, 0.0}, 0.0), vec3{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, 500 * mass, 500 * (10.0 / 12) * inertia_tensor, STATIC);
    world.attach_box_collider(plane_id, {50.0, 0.2, 50.0});

    // world.attach_sphere_collider(ball_id, 1.0);
    int i = 0;
    for (const auto &position : ball_positions)
    {
        int ball_id = world.create_body(position, ti::quat_from_axis_angle({1.0, 0.0, 0.0}, PI / 2), vec3{0.0, 0.01, 0.0}, ang_vel, 1 * mass, mass * cursed_cylinder_inertia, DYNAMIC);
        if (i % 2 == 0)
        {
            world.attach_box_collider(ball_id, {RADIUS, 1.5 * RADIUS, RADIUS});
        }
        else if (i % 2 == 1)
        {
            world.attach_sphere_collider(ball_id, RADIUS);
        }
        else
        {
            world.attach_cylinder_collider(ball_id, RADIUS, 1.5 * RADIUS);
        }
        // world.attach_sphere_collider(ball_id, RADIUS);
        i++;
    }

    int ball_id = world.create_body({-10.0, 2.0, 0.0}, ti::quat_from_axis_angle({0.0, 0.0, 1.0}, 0.0), vec3{20.1, 0.0, 0.0}, {0.0, 0.0, 0.0}, 50 * mass, 5.0 * (10.0 / 12) * inertia_tensor, DYNAMIC);
    world.attach_sphere_collider(ball_id, 2.0 * RADIUS);

    world.set_gravity({0.0, -9.8, 0.0});
    

    return world;

}