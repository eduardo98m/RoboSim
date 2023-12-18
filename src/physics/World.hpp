#pragma once
#include "physics/math/math.hpp"
#include "bodies/Body.hpp"
#include "constraints/PositionalConstraint.hpp"
#include "constraints/RotationalConstraint.hpp"
#include "constraints/RevoluteJointConstraint.hpp"
#include <vector>

// Create a blank class called World
namespace robosim
{
    class World
    {
    private:
        scalar timestep;
        int substeps;

        std::vector<Body> bodies;
        std::vector<PositionalConstraint> positional_constraints;
        std::vector<RotationalConstraint> rotational_constraints;
        std::vector<RevoluteJointConstraint> revolute_joint_constraints;

        void update_bodies_position_and_orientation(scalar h);
        void solve_positions(scalar inv_h);
        void update_bodies_velocities(scalar inv_h);
        void solve_velocities(scalar inv_h);

    public:
        // Constructor
        World(scalar timestep = 0.01, int substeps = 20);
        // Destructor
        //~World();

        // Get the time step of the world
        scalar get_time_step();

        void set_gravity(vec3 gravity);

        int add_body(Body body);

        vec3 get_body_position(int id);
        quat get_body_orientation(int id);

        //
        int create_positional_constraint(int body_1_id,
                                          int body_2_id,
                                          vec3 r_1,
                                          vec3 r_2,
                                          scalar compliance = 0.0,
                                          scalar damping = 0.0);

        int create_revolute_constraint(int body_1_id,
                                        int body_2_id,
                                        vec3 aligned_axis,
                                        vec3 r_1,
                                        vec3 r_2,
                                        scalar compliance = 0.0,
                                        scalar damping = 0.0,
                                        RevoluteJointType type = FREE,
                                        bool limited = false,
                                        scalar lower_limit = 0.0,
                                        scalar upper_limit = 0.0);

        int create_rotational_constraint(int body_1_id,
                                          int body_2_id,
                                          vec3 r_1,
                                          vec3 r_2,
                                          scalar compliance = 0.0,
                                          scalar damping = 0.0);

        int add_rotational_constraint(RotationalConstraint cosntraint);

        void step();
    };
} // namespace robosim