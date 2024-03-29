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
    Body *body_1;
    // Reference to the second body
    Body *body_2;
    // Fricction properties
    scalar static_fricction_coeff;
    scalar dynamic_fricction_coeff;
    scalar restitution;

    // Positional Constraints (Normal and tangencial)
    PositionalConstraint * normal_constraint;
    PositionalConstraint * tangencial_constraint;

    scalar relative_velocity = 0.0; //= {0.0, 0.0, 0.0, 0.0};

    vec3 normal_force = (vec3){0.0, 0.0, 0.0};

public:
    // Collision points and normal vector.
    ContactPoint collision_response;
    bool broad_phase_detection;
    bool collision;
    ContactConstraint(Body *body_1, Body *body_2);
    void check_broad_phase(scalar time_step);
    void calculate_narrow_phase_collision_response(void);
    AABB get_aabb(Body *body);
    void apply_constraint(scalar inverse_time_step);
    void apply_constraint_velocity_level(scalar time_step);
    void reset_lagrange_multipliers(void);
};