#pragma once
#include "physics/math/math.hpp"
#include "physics/bodies/Body.hpp"
#include "PositionalConstraint.hpp"
#include "RotationalConstraint.hpp"
#include <memory>
#include "Utils.hpp"
#include "JointsCommon.hpp"


struct FixedJointInfo{
    vec3 position;
    vec3 moving_axis;
    vec3 force;
    vec3 torque;
    scalar current_position;
};

class FixedJointConstraint{
    public:
        // Reference to the first body
        std::shared_ptr<Body> body_1;
        // Reference to the second body
        std::shared_ptr<Body> body_2;

        // Constraint position relative to the first body
        vec3 r_1;
        // Constraint position relative to the second body
        vec3 r_2;

        // Force and torques (Wrenches)
        vec3 force;
        vec3 torque;
        
        // Constraints
        std::shared_ptr<RotationalConstraint> aligned_constraint;
        
        std::shared_ptr<PositionalConstraint> attachment_point_constraint;
    public:

        FixedJointConstraint(std::shared_ptr<Body> body_1, 
                             std::shared_ptr<Body> body_2,
                             vec3 r_1 = vec3{0.0, 0.0, 0.0}, 
                             vec3 r_2 = vec3{0.0, 0.0, 0.0}
                             );

        void apply_constraint(scalar inverse_time_step, scalar time_step);

        void reset_lagrange_multipliers(void);

        FixedJointInfo get_info(void);

};