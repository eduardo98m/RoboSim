#pragma once
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

robosim::World urdf_scenario()
{

    robosim::World world = robosim::World(0.01, 60);

    scalar mass = 1.0;
    mat3 inertia_tensor = mat3{1.0, 0.0, 0.0,
                               0.0, 1.0, 0.0,
                               0.0, 0.0, 1.0};
    
    vec3 ang_vel = vec3{0.0, 0.0, 0.0};
    vec3 lin_vel = {0.0, 0.0, 0.0};
    quat ori = ti::quat_from_axis_angle({1.0, 0.0, 0.0}, -PI/2);
    

    //size_t cyl_id = world.load_urdf("../resources/cyllinder.urdf", {0.0, 0.0, 0.0});

    size_t spot_id = world.load_urdf("../resources/giadog/mini_ros/urdf/spot.urdf", {0.0, 2.0, 0.0}, ori);
    
    //size_t articulated_system_id = world.load_urdf("../resources/husky.urdf", {0.0, 5.0, 0.0});

    // size_t aminitaur_id = world.load_urdf("../resources/minitaur.urdf", {5.0, 00.0, 0.0});

    //size_t r2d2_id = world.load_urdf("../resources/r2d2.urdf", {-5.0, 00.0, 0.0});

    size_t plane_id = world.create_body({0.0, 0.0, 0.0}, ti::quat_from_axis_angle({0.0, 1.0, 0.0}, 0.0), vec3{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, 500 * mass, 500 * (10.0 / 12) * inertia_tensor, STATIC);
    world.attach_box_collider(plane_id, {50.0, 0.2, 50.0});
    world.attach_box_visual_shape(plane_id, {50.0, 0.2, 50.0});

    world.set_gravity({0.0, -9.8, 0.0});

    return world;
}