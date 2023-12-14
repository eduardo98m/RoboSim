#pragma once
#include "math/math.hpp"

// Create a blank class called World
namespace robosim{
    class World {
        public:
            // Constructor
            World();
            // Destructor
            ~World();

            // // Remove objects from the world
            // void remove_object(Object * object);

            // Get the time step of the world
            scalar get_time_step();

            // Get the contacts of the world
            //std::vector<Contact> get_contacts();
    };
} // namespace robosim