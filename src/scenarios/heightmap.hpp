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


robosim::World heightmap_scenario(){
    
    robosim::World world = robosim::World(0.01, 20);
    quat ori = ti::quat_from_axis_angle({0.0, 0.0, 1.0}, 0.0);
    scalar mass = 1.0;
    mat3 inertia_tensor = mat3{1.0, 0.0, 0.0,
                               0.0, 1.0, 0.0,
                               0.0, 0.0, 1.0};
    

    // Create the floor with a heightmap collider
    size_t x_dims = 50; 
    size_t y_dims = 50;
    std::vector<scalar> heightdata(x_dims * y_dims);
    for (size_t i = 0; i < x_dims * y_dims; ++i) {
        heightdata[i] = 0.4;
    }
    int floor_id = world.create_body({0.0, 0.0, 0.0}, 
                                     ti::quat_from_axis_angle({1.0, 0.0, 0.0}, -PI/2), 
                                     vec3{0.0, 0.0, 0.0}, 
                                     {0.0, 0.0, 0.0}, 1.0 * mass, inertia_tensor, BodyType::STATIC);
    world.attach_heightmap_collider(floor_id, x_dims, y_dims, heightdata, x_dims, y_dims);

    
    int id = world.create_body({5.0, 4.0, 0.0}, ori, vec3{-0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, 1.0 *mass);
    world.attach_sphere_collider(id, 2.1);

    rs::Color color = {25, 41, 55, 0};
    world.set_body_color(id, color);

    world.set_gravity({0.0, -9.8, 0.0});
    


    return world;
}