#pragma once

#include "physics/math/math.hpp"
#include "physics/bodies/Body.hpp"
#include "PositionalConstraint.hpp"
#include "collisions.hpp"

class ContactConstraint{
    private:
        // Reference to the first body
        Body *body_1;
        // Reference to the second body
        Body *body_2;
        // Fricction properties
        scalar static_fricction_coeff;
        scalar dynamic_fricction_coeff;

        // Positional Constraints (Normal and tangencial)
        PositionalConstraint *normal_constraint;
        PositionalConstraint *tangencial_constraint;
    
    public:
        ContactConstraint(Body *body_1, Body *body_2);
        void apply_constraint(const CollisionResponseData &collision_response, scalar inverse_time_step);
        void reset_lagrange_multipliers(void);
};