#pragma once

#include "scenarios/scenario.hpp"
#include "physics/math/math.hpp"
#include "physics/constraints/RevoluteJointConstraint.hpp"

// Import the sine function from the standard library
#include <math.h>
#include <iostream>
#include <raylib.h>
#include "physics/World.hpp"
#include "Interface.hpp"
#include <memory>
#include <random>

robosim::World box_stack_scenario()
{
    const float Y_MIN = 0.1f, Y_MAX = 5.1f;
    const float HALF_LENGHT = 0.4f;
    const int NUM_BOXES = 6; // Number of boxes in the column

    // Define the dimensions of the plane
    const float PLANE_HEIGHT = 0.2f;

    mat3 inertia_tensor = mat3{1.0, 0.0, 0.0,
                               0.0, 1.0, 0.0,
                               0.0, 0.0, 1.0};

    robosim::World world = robosim::World(0.01, 60);

    // Create the plane
    int plane_id = world.create_body({0.0, PLANE_HEIGHT / 2, 0.0}, 
                                     ti::quat_from_axis_angle({1.0, 0.0, 0.0}, 0.0), 
                                     vec3{0.0, 0.0, 0.0}, 
                                     vec3{0.0, 0.0, 0.0}, 
                                     0.0, 
                                     inertia_tensor, 
                                     STATIC);
    world.attach_box_collider(plane_id, {10.0f, PLANE_HEIGHT, 10.0f});

    // Generate positions for the boxes in a column
    float stepY = (Y_MAX - Y_MIN) / NUM_BOXES;
    for (int i = 0; i < NUM_BOXES; ++i)
    {
        float x = 0.0f; // Centered in X
        float y = Y_MIN + i * stepY + stepY / 2 + PLANE_HEIGHT + 0.01; // Stack boxes on top of each other starting from the plane's height
        float z = 0.0f; // Centered in Z
        vec3 position = {x, y, z};

        int box_id = world.create_body(position);
        world.attach_box_collider(box_id, {HALF_LENGHT, HALF_LENGHT, HALF_LENGHT});
    }

    world.set_gravity({0.0, -9.8, 0.0});
    

    return world;
}