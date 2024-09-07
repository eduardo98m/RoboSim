#pragma once

#include "physics/math/math.hpp"
#include "physics/bodies/Body.hpp"
#include "PositionalConstraint.hpp"
#include "collisions.hpp"
#include "hpp/fcl/distance.h"
#include "broad_phase.hpp"
#include <utility>

class ContactConstraint
{
private:
    // Reference to the first body
    std::shared_ptr<Body> body_1;
    // Reference to the second body
    std::shared_ptr<Body> body_2;

    // Material properties of the contact
    // (Calculated by combining the material proerties of the colliders)
    // Static fricction
    scalar static_fricction;
    // Dynamic friction of the contact
    scalar dynamic_fricction;
    // Restitution coefficient of the contact
    scalar restitution;

    //

    // Relevant quantities
    scalar relative_velocity = 0.0; //= {0.0, 0.0, 0.0, 0.0};
    scalar normal_constraint_lagrange_multiplier = 0.0;
    scalar tangencial_constraint_lagrange_multiplier = 0.0;
    vec3 normal_force = (vec3){0.0, 0.0, 0.0};
    vec3 tangencial_force = (vec3){0.0, 0.0, 0.0};

public:
    // Check if the two bodies are really colliding
    bool collision;
    // Normal Vector of the contact
    vec3 normal;
    // Contact point in the first body
    vec3 p_1;
    // Contact point in the second body
    vec3 p_2;

    ContactConstraint(std::shared_ptr<Body> body_1,
                      std::shared_ptr<Body> body_2,
                      vec3 normal,
                      vec3 p_1,
                      vec3 p_2,
                      scalar static_fricction,
                      scalar dynamic_fricction,
                      scalar restitution);

    void solve_normal_constraint(scalar inverse_time_step, scalar magnitude);
    void solve_tangencial_constraint(scalar inverse_time_step);
    void calculate_narrow_phase_collision_response(void);
    void apply_constraint_position_level(scalar inverse_time_step);
    void apply_constraint_velocity_level(scalar time_step);
    void reset_lagrange_multipliers(void);
};