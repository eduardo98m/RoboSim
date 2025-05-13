#pragma once
#include "physicsAcc/Constraints/ConstraintCollection.hpp"

/**
 * @brief Parameters for creating a constraint between two bodies.
 *
 * @param body_1 Index of the first body.
 * @param body_2 Index of the second body.
 * @param type  Type of the constraint (POSITIONAL or ROTATIONAL).
 * @param r_1   Local position on body 1 (default: (0,0,0)).
 * @param r_2   Local position on body 2 (default: (0,0,0)).
 * @param compliance Constraint compliance (default: 0.0).
 */
struct ConstraintsParams
{
  size_t body_1;
  size_t body_2;
  ConstraintType type;
  vec3 r_1 = (vec3){0.0, 0.0, 0.0};
  vec3 r_2 = (vec3){0.0, 0.0, 0.0};
  scalar compliance = 0.0; 
};



/**
 * @brief Creates a new constraint in the given constraint collection.
 * 
 * @param cc The constraint collection to add the constraint to.
 * @param params The parameters of the constraint to create.
 * @return The ID of the newly created constraint.
 */
size_t create_constraint(ConstraintCollection &cc, ConstraintsParams params){
    size_t id = cc.n_constraints;
    cc.n_constraints += 1;
    cc.body_1.push_back(params.body_1);
    cc.body_2.push_back(params.body_2);
    cc.r_1.push_back(params.r_1);
    cc.r_2.push_back(params.r_2);
    cc.direction.push_back((vec3){1.0, 0.0, 0.0});
    cc.magnitude.push_back(0.0);
    cc.lambda.push_back(0.0);
    cc.force.push_back((vec3){0.0, 0.0, 0.0});
    cc.torque.push_back((vec3){0.0, 0.0, 0.0});
    cc.compliance.push_back(params.compliance);
    cc.type.push_back(params.type);
    cc.impulse.push_back((vec3){0.0, 0.0, 0.0});
    return id;
}
