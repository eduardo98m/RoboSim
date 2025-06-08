#pragma once
#include "physicsAcc/Constraints/Joint.hpp"
#include "physicsAcc/API/ConstraintsAPI.hpp"

struct PrismaticJointParams
{
    size_t body_1;
    size_t body_2;
    vec3 moving_axis = (vec3){1.0, 0.0, 0.0}; // Axis
    vec3 r_1 = (vec3){0.0, 0.0, 0.0};         // Default being the COM of the object
    vec3 r_2 = (vec3){0.0, 0.0, 0.0};         // Default being the COM of the object
    JointActuationType actuation_type = JointActuationType::FREE;
    scalar compliance = 0.0; // Inverse of the stiffness
    scalar damping = 0.0;    //
    bool limited = false;
    scalar lower_limit = 0.0;
    scalar upper_limit = 0.0;
};

struct RevoluteJointParams
{
    size_t body_1;
    size_t body_2;
    vec3 aligned_axis = (vec3){1.0, 0.0, 0.0}; // Aligned Axis (Main axis)
    vec3 limit_axis = (vec3){0.0, 1.0, 0.0};   // Limit Axis
    vec3 r_1 = (vec3){0.0, 0.0, 0.0};          // Default being the COM of the object
    vec3 r_2 = (vec3){0.0, 0.0, 0.0};          // Default being the COM of the object
    JointActuationType actuation_type = JointActuationType::FREE;
    scalar compliance = 0.0; // Inverse of the stiffness
    scalar damping = 0.0;    //
    bool limited = false;
    scalar lower_limit = 0.0;
    scalar upper_limit = 0.0;
};

struct FixedJointParams
{
    size_t body_1;
    size_t body_2;
    vec3 r_1 = (vec3){0.0, 0.0, 0.0}; // Default being the COM of the object
    vec3 r_2 = (vec3){0.0, 0.0, 0.0}; // Default being the COM of the object
};

/**
 * @brief Creates a primsatic joint between two bodies.
 *
 * This function creates a primsatic joint, which allows straight movement on a single axis,
 * between two specified bodies. It involves creating multiple constraints in the
 * ConstraintCollection to simulate the joint's behavior.
 *
 * @param jc The JointCollection to store the joint data.
 * @param cc The ConstraintCollection to store the constraints associated with the joint.
 * @param params The parameters defining the primsatic joint.
 * @return The ID of the created joint.
 */
size_t create_prismatic_joint(JointCollection &jc, ConstraintCollection &cc, PrismaticJointParams params)
{

    // Check the correct order
    // Alignment
    size_t c_1 = create_constraint(cc, ConstraintsParams{
                                           .body_1 = params.body_1,
                                           .body_2 = params.body_2,
                                           .type = ConstraintType::ROTATIONAL,
                                           .r_1 = params.r_1,
                                           .r_2 = params.r_2,
                                           .compliance = 0.0});

    // Attachment
    size_t c_2 = create_constraint(cc, ConstraintsParams{
                                           .body_1 = params.body_1,
                                           .body_2 = params.body_2,
                                           .type = ConstraintType::POSITIONAL,
                                           .r_1 = params.r_1,
                                           .r_2 = params.r_2,
                                           .compliance = 0.0});

    // Drive
    size_t c_3 = create_constraint(cc, ConstraintsParams{
                                           .body_1 = params.body_1,
                                           .body_2 = params.body_2,
                                           .type = ConstraintType::POSITIONAL,
                                           .r_1 = params.r_1,
                                           .r_2 = params.r_2,
                                           .compliance = params.compliance,

                                       });

    jc.constraint_count.push_back(3);
    jc.constraint_start.push_back(c_1);

    size_t id = jc.n_joints;
    jc.n_joints += 1;

    jc.body_1.push_back(params.body_1);
    jc.body_2.push_back(params.body_2);
    jc.r_1.push_back(params.r_1);
    jc.r_2.push_back(params.r_2);
    jc.type.push_back(JointType::PRISMATIC);
    jc.actuation_type.push_back(params.actuation_type);
    jc.limited.push_back(params.limited);
    jc.lower_limit.push_back(params.lower_limit);
    jc.upper_limit.push_back(params.upper_limit);
    jc.main_axis.push_back(params.moving_axis);
    jc.limit_axis.push_back(vec3(0.0, 0.0, 0.0)); // Pading
    jc.current_position.push_back(0.0);
    jc.damping.push_back(params.damping);

    jc.target_speed.push_back(0.0);
    jc.target_position.push_back(0.0);

    return id;
}

/**
 * @brief Creates a revolute joint between two bodies.
 *
 * This function creates a revolute joint, which allows rotation around a single axis,
 * between two specified bodies. It involves creating multiple constraints in the
 * ConstraintCollection to simulate the joint's behavior.
 *
 * @param jc The JointCollection to store the joint data.
 * @param cc The ConstraintCollection to store the constraints associated with the joint.
 * @param params The parameters defining the revolute joint.
 * @return The ID of the created joint.
 */
size_t create_revolute_joint(JointCollection &jc, ConstraintCollection &cc, RevoluteJointParams params)
{
    // Alignment constraint
    size_t c_1 = create_constraint(cc, ConstraintsParams{
                                           .body_1 = params.body_1,
                                           .body_2 = params.body_2,
                                           .type = ConstraintType::ROTATIONAL,
                                           .r_1 = params.r_1,
                                           .r_2 = params.r_2,
                                           .compliance = 0.0,
                                       });

    // Attachment constraint
    size_t c_2 = create_constraint(cc, ConstraintsParams{
                                           .body_1 = params.body_1,
                                           .body_2 = params.body_2,
                                           .type = ConstraintType::POSITIONAL,
                                           .r_1 = params.r_1,
                                           .r_2 = params.r_2,
                                           .compliance = 0.0,
                                       });

    // Limit constraint
    size_t c_3 = create_constraint(cc, ConstraintsParams{
                                           .body_1 = params.body_1,
                                           .body_2 = params.body_2,
                                           .type = ConstraintType::ROTATIONAL,
                                           .r_1 = params.r_1,
                                           .r_2 = params.r_2,
                                           .compliance = 0.0,
                                       });

    // Drive constraint
    size_t c_4 = create_constraint(cc, ConstraintsParams{
                                           .body_1 = params.body_1,
                                           .body_2 = params.body_2,
                                           .type = ConstraintType::ROTATIONAL,
                                           .r_1 = params.r_1,
                                           .r_2 = params.r_2,
                                           .compliance = params.compliance

                                       });

    jc.constraint_count.push_back(4);
    jc.constraint_start.push_back(c_1);

    size_t id = jc.n_joints;
    jc.n_joints += 1;

    jc.body_1.push_back(params.body_1);
    jc.body_2.push_back(params.body_2);
    jc.r_1.push_back(params.r_1);
    jc.r_2.push_back(params.r_2);
    jc.type.push_back(JointType::REVOLUTE);
    jc.actuation_type.push_back(params.actuation_type);
    jc.limited.push_back(params.limited);
    jc.lower_limit.push_back(params.lower_limit);
    jc.upper_limit.push_back(params.upper_limit);
    jc.main_axis.push_back(params.aligned_axis);
    jc.limit_axis.push_back(params.limit_axis);
    jc.damping.push_back(params.damping);
    jc.target_position.push_back(0.0);  // Pading
    jc.target_speed.push_back(0.0); // Pading
    jc.current_position.push_back(0.0);

    return id;
}

/**
/**
 * @brief Creates a fixed joint between two bodies.
 *
 * This function creates a fixed joint, which prevents any relative movement
 * between two specified bodies. It involves creating constraints in the
 * ConstraintCollection to simulate the joint's behavior.
 *
 * @param jc The JointCollection to store the joint data.
 * @param cc The ConstraintCollection to store the constraints associated with the joint.
 * @param params The parameters defining the fixed joint.
 * @return The ID of the created joint.
 */
size_t create_fixed_joint(JointCollection &jc, ConstraintCollection &cc, FixedJointParams params)
{
    // Alignment constraint
    size_t c_1 = create_constraint(cc, ConstraintsParams{
                                           .body_1 = params.body_1,
                                           .body_2 = params.body_2,
                                           .type = ConstraintType::ROTATIONAL,
                                           .r_1 = params.r_1,
                                           .r_2 = params.r_2,
                                           .compliance = 0.0

                                       });

    // Attachment constraint
    size_t c_2 = create_constraint(cc, ConstraintsParams{
                                           .body_1 = params.body_1,
                                           .body_2 = params.body_2,
                                           .type = ConstraintType::POSITIONAL,
                                           .r_1 = params.r_1,
                                           .r_2 = params.r_2,
                                           .compliance = 0.0});

    jc.constraint_count.push_back(2);
    jc.constraint_start.push_back(c_1);

    size_t id = jc.n_joints;
    jc.n_joints += 1;

    jc.body_1.push_back(params.body_1);
    jc.body_2.push_back(params.body_2);
    jc.r_1.push_back(params.r_1);
    jc.r_2.push_back(params.r_2);
    jc.type.push_back(JointType::FIXED);
    jc.actuation_type.push_back(JointActuationType::FREE); // Fixed joints don't have actuation
    jc.limited.push_back(false);                           // Fixed joints don't have limits
    jc.lower_limit.push_back(0.0);
    jc.upper_limit.push_back(0.0);

    jc.main_axis.push_back(vec3(0.0, 0.0, 0.0));  // Padding
    jc.limit_axis.push_back(vec3(0.0, 0.0, 0.0)); // Padding
    jc.current_position.push_back(0.0);           // Padding

    jc.damping.push_back(0.0);
    jc.target_speed.push_back(0.0); // Pading
    jc.target_position.push_back(0.0);  // Pading

    return id;
}
