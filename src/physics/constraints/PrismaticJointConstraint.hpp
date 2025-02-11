#pragma once
#include "physics/math/math.hpp"
#include "physics/bodies/Body.hpp"
#include "PositionalConstraint.hpp"
#include "RotationalConstraint.hpp"
#include <memory>
#include "Utils.hpp"
#include "JointsCommon.hpp"


struct PrismaticJointInfo{
    vec3 position;
    vec3 moving_axis;
    vec3 force;
    vec3 torque;
    scalar current_position;
    bool limited;
    scalar upper_limit;
    scalar lower_limit;
};

class PrismaticJointConstraint{
    public:
        // Reference to the first body
        std::shared_ptr<Body> body_1;
        // Reference to the second body
        std::shared_ptr<Body> body_2;

        // Axes of the first and second body
        vec3 moving_axis;
        
        // Constraint position relative to the first body
        vec3 r_1;
        // Constraint position relative to the second body
        vec3 r_2;

        // Compliance 
        scalar compliance;
        // Damping 
        scalar damping;

        
        scalar target_speed = 0.0;
        scalar target_position = 0.0;
        
        // Limited Joint
        bool limited;
        scalar upper_limit;
        scalar lower_limit;

        // Force and torques (Wrenches)
        vec3 force;
        vec3 torque;

        // Current angle:
        scalar current_position;
        // Previous angle:
        scalar previous_position;

        // Constraints
        std::shared_ptr<RotationalConstraint> aligned_constraint;
        std::shared_ptr<PositionalConstraint> drive_joint_constraint;
        
        std::shared_ptr<PositionalConstraint> attachment_point_constraint;
    public:
        // Joint type (FREE - DRIVEN_BY_POSITION - DRIVEN_BY_SPEED)
        JointControlType type;

        PrismaticJointConstraint(std::shared_ptr<Body> body_1, 
                             std::shared_ptr<Body> body_2,
                             vec3 moving_axis = vec3(1.0, 0.0, 0.0),
                             vec3 r_1 = vec3{0.0, 0.0, 0.0}, 
                             vec3 r_2 = vec3{0.0, 0.0, 0.0},
                             scalar compliance = 0.0, 
                             scalar damping = 0.0,
                             JointControlType type = JointControlType::FREE,
                             bool limited = false,
                             scalar lower_limit = 0.0 ,
                             scalar upper_limit = 0.0);

        void apply_constraint(scalar inverse_time_step, scalar time_step);

        void apply_joint_damping(scalar time_step);

        void set_traget_position(scalar angle);

        void set_target_speed(scalar speed);

        void reset_lagrange_multipliers(void);

        PrismaticJointInfo get_info(void);

        scalar get_current_position(void);

};