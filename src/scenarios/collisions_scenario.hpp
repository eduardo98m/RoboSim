#pragma once

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


robosim::World collisions_scenario(){

    
    const int s = 1000; // 10x10x10 grid
    const float X_MIN = -1.5f, X_MAX = 1.5f;
    const float Y_MIN = 1.1f, Y_MAX = 3.1f;
    const float Z_MIN = -1.5f, Z_MAX = 1.5f;
    const float RADIUS = 0.2f;
    const int GRID_SIZE = 4;
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
    for (int i = 0; i < GRID_SIZE; ++i) {
        for (int j = 0; j < GRID_SIZE; ++j) {
            for (int k = 0; k < GRID_SIZE; ++k) {
                float x = X_MIN + i * stepX + stepX / 2;
                float y = Y_MIN + j * stepY + stepY / 2;
                float z = Z_MIN + k * stepZ + stepZ / 2;
                ball_positions.push_back({x, y, z});
            }
        }
    }

    robosim::World world = robosim::World(0.01, 60);
    quat ori = ti::quat_from_axis_angle({0.0, 0.0, 1.0}, 2.0);
    scalar mass = 1.0;
    mat3 inertia_tensor = mat3{1.0, 0.0, 0.0,
                               0.0, 1.0, 0.0,
                               0.0, 0.0, 1.0};
    
    vec3 ang_vel = vec3{0.0, 1.1, 0.0};

    uint plane_id = world.add_plane({0.0, 1.0, 0}, 0.0);
    for (const auto& position : ball_positions) {
        int ball_id = world.create_body(position, ori, vec3{0.0, 0.0, 0.01}, ang_vel, 1*mass, ( 10.0/12) * inertia_tensor, DYNAMIC);
        //world.set_body_box_collider(ball_id, {RADIUS,RADIUS,RADIUS});
        world.set_body_sphere_collider(ball_id, RADIUS);
    }

    int ball_id = world.create_body({5.0, 2.0, 0.0}, ori, vec3{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, 4*mass, 4.0 *  (10.0/12) * inertia_tensor, DYNAMIC);
    world.set_body_sphere_collider(ball_id, 1.0);

    world.set_gravity({0.0, -9.8, 0.0});
    world.collisions_detection_preparations();


    return world;


}