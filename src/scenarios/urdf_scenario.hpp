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

    robosim::World world = robosim::World(0.01, 120);
    

    size_t articulated_system_id = world.load_urdf("../resources/giadog/mini_ros/urdf/spot.urdf", {0.0, 10.0, 0.0});

    world.add_plane({0.0, 1.0, 0.0}, 0.0);

    //

    //world.set_gravity({0.0, 0.0, 0.0});

    return world;
}