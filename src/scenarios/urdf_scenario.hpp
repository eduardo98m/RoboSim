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
    

    size_t cyl_id = world.load_urdf("../resources/cyllinder.urdf", {0.0, 0.0, 0.0});

    //size_t spot_id = world.load_urdf("../resources/spot.urdf", {0.0, 00.0, 5.0});
    
    size_t articulated_system_id = world.load_urdf("../resources/husky.urdf", {0.0, 5.0, 0.0});

    // size_t aminitaur_id = world.load_urdf("../resources/minitaur.urdf", {5.0, 00.0, 0.0});

    //size_t r2d2_id = world.load_urdf("../resources/r2d2.urdf", {-5.0, 00.0, 0.0});

    return world;
}