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


robosim::World simple_box_collisions_scenario(){
    
    robosim::World world = robosim::World(0.01, 1);
    quat ori = ti::quat_from_axis_angle({0.0, 0.0, 1.0}, 2.0);
    scalar mass = 1.0;
    mat3 inertia_tensor = mat3{1.0, 0.0, 0.0,
                               0.0, 1.0, 0.0,
                               0.0, 0.0, 1.0};
    

    world.add_plane({0.0, 1.0, 0.0}, 0.0);

    
    int id = world.create_body({5.0, 4.0, 0.0}, ori, vec3{-0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, 1.0 *mass);
    world.attach_box_collider(id, {1.0, 1.0, 1.0});

    world.attach_mesh_visual_shape(id, "../resources/coral/coral.obj");
    //world.set_visual_shape_color(id, (rs::Color){25, 41, 55, 0});

    world.set_gravity({0.0, -9.8, 0.0});

    return world;
}