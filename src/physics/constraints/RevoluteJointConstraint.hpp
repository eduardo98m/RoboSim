#pragma once
#include "physics/math/math.hpp"
#include "physics/bodies/Body.hpp"
#include "PositionalConstraint.hpp"
#include "RotationalConstraint.hpp"
#include "Utils.hpp"

enum RevoluteJointType{
    FREE,
    DRIVEN_BY_POSITION,
    DRIVEN_BY_SPEED
};

struct RevoluteJointInfo{
    vec3 position;
    vec3 rotation_axis;
    vec3 body_1_limit_axis;
    vec3 body_2_limit_axis;
    vec3 force;
    vec3 torque;
    scalar current_angle;
};

class RevoluteJointConstraint{
    private:
        // Reference to the first body
        Body *body_1;
        // Reference to the second body
        Body *body_2;

        // Axes of the first and second body
        vec3 aligned_axis;
        vec3 limit_axis;

        // Constraint position relative to the first body
        vec3 r_1;
        // Constraint position relative to the second body
        vec3 r_2;

        // Compliance 
        scalar compliance;
        // Damping 
        scalar damping;

        
        scalar target_speed = 0.0;
        scalar target_angle = 0.0;
        
        // Limited Joint
        bool limited;
        scalar upper_limit;
        scalar lower_limit;

        // Force and torques (Wrenches)
        vec3 force;
        vec3 torque;

        // Current angle:
        scalar current_angle;
        // Previous angle:
        scalar previous_angle;

        // Constraints
        RotationalConstraint *aligned_constraint;
        RotationalConstraint *angle_limit_constraint;
        RotationalConstraint *drive_joint_constraint;
        
        PositionalConstraint *attachment_point_constraint;
    public:
        // Joint type (FREE - DRIVEN_BY_POSITION - DRIVEN_BY_SPEED)
        RevoluteJointType type;

        RevoluteJointConstraint(Body * body_1, 
                             Body * body_2,
                             vec3 aligned_axis = vec3(1.0, 0.0, 0.0),
                             vec3 limit_axis = vec3(0.0, 1.0, 0.0),
                             vec3 r_1 = vec3{0.0, 0.0, 0.0}, 
                             vec3 r_2 = vec3{0.0, 0.0, 0.0},
                             scalar compliance = 0.0, 
                             scalar damping = 0.0,
                             RevoluteJointType type = FREE,
                             bool limited = false,
                             scalar lower_limit = 0.0 ,
                             scalar upper_limit = 0.0);

        void apply_constraint(scalar inverse_time_step, scalar time_step);

        void apply_joint_damping(scalar time_step);

        void set_traget_angle(scalar angle);

        void set_target_speed(scalar speed);

        void reset_lagrange_multipliers(void);

        RevoluteJointInfo get_info(void);

        scalar get_current_angle(void);

};