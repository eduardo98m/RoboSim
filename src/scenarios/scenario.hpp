#pragma once
#include "physics/World.hpp"
#include "Visualizer.hpp"
#include <memory>

struct Scenario {
    robosim::World world;
    std::function<void(std::shared_ptr<robosim::World>, std::shared_ptr<Visualizer>)> step_callback = [](std::shared_ptr<robosim::World>, std::shared_ptr<Visualizer>){};
    std::function<void()> gui_callback = [](){};
};