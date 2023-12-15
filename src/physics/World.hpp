#pragma once
#include "math/math.hpp"
#include "bodies/Body.hpp"
#include "constraints/PositionalConstraint.hpp"
#include "constraints/RotationalConstraint.hpp"
#include <vector>

// Create a blank class called World
namespace robosim{
    class World {
        private:
            scalar timestep;
            uint substeps;

            std::vector<Body> bodies;
            std::vector<PositionalConstraint> positional_constraints;
            std::vector<RotationalConstraint> rotational_constraints;
        

            void update_bodies_position_and_orientation(scalar h);
            void solve_positions(scalar inv_h);
            void update_bodies_velocities(scalar inv_h);
            void solve_velocities(scalar inv_h);

        public:
            // Constructor
            World(scalar timestep = 0.01, uint substeps = 20);
            // Destructor
            //~World();


            // Get the time step of the world
            scalar get_time_step();

            void set_gravity(vec3 gravity);

            uint add_body(Body body);

            vec3 get_body_position(uint id);
            quat get_body_orientation(uint id);

            // 
            uint create_positional_constraint(uint body_1_id, uint body_2_id, vec3 r_1, vec3 r_2, scalar compliance = 0.0, scalar damping = 0.0);

            uint add_rotational_constraint(RotationalConstraint cosntraint);

            void step();      
    };
} // namespace robosim