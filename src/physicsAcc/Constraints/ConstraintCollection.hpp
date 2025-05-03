#pragma once
#include "physics/math/math.hpp"
#include "physicsAcc/Body/BodyCollection.hpp"
#include <vector>
#include <tuple>

/**
 * @brief Types of constraints 
 */
enum ConstraintType
{
    POSITIONAL,  // Constraint enforcing positional relationships 
    ROTATIONAL   // Constraint enforcing rotational relationships 
};

/**
 * @brief Collection of per-constraint data used by the physics solver.
 *
 * Holds all data arrays needed to evaluate and apply constraint impulses
 * between pairs of bodies.
 */
struct ConstraintCollection
{
    size_t n_constraints;             // Number of constraints in the collection
    std::vector<size_t> body_1;       // Index of the first body in each constraint
    std::vector<size_t> body_2;       // Index of the second body in each constraint
    std::vector<vec3> r_1;            // Local contact point on body 1
    std::vector<vec3> r_2;            // Local contact point on body 2
    std::vector<vec3> direction;      // Constraint direction (unit vector)
    std::vector<scalar> magnitude;    // Current constraint magnitude
    std::vector<scalar> lambda;       // Lagrange multiplier per constraint
    std::vector<vec3> force;          // Positional constraint force
    std::vector<vec3> torque;         // Rotational constraint torque
    std::vector<scalar> compliance;   // Constraint compliance (softness)
    std::vector<ConstraintType> type; // Position vs. rotation constraint
    std::vector<vec3> impulse;        // Constraint impulse applied
};

/**
 * @brief Computes and stores magnitude and direction for a raw constraint vector.
 *
 * @param cc    The ConstraintCollection to update.
 * @param i     Index of the constraint.
 * @param value Raw 3D vector whose magnitude/direction to extract.
 */
void set_value(ConstraintCollection &cc, size_t i, vec3 value);

/**
 * @brief Computes the change in the Lagrange multiplier for one constraint.
 *
 * Implements: Δλ = (–b – α·λ_old) / (w₁ + w₂ + α)
 *
 * @param cc                The ConstraintCollection.
 * @param i                 Index of the constraint.
 * @param w_1               Generalized inverse mass of body 1.
 * @param w_2               Generalized inverse mass of body 2.
 * @param inverse_time_step Inverse of the time step (1/Δt).
 * @return                  Change in Lagrange multiplier Δλ.
 */
scalar compute_delta_lambda(const ConstraintCollection &cc, size_t i, scalar w_1, scalar w_2, scalar inverse_time_step);

/**
 * @brief Solves and applies a rotational constraint impulse between two bodies.
 *
 * - Computes generalized inverse masses for each body about the constraint axis.
 * - Computes Δλ and updates the constraint’s λ.
 * - Converts Δλ into an impulse and corresponding torque.
 * - Applies equal-and-opposite rotational impulses to each body.
 *
 * @param bc                The BodyCollection containing all bodies.
 * @param cc                The ConstraintCollection.
 * @param i                 Index of the constraint.
 * @param inverse_time_step Inverse of the time step (1/Δt).
 */
void compute_rotational_constraint_impulse(BodyCollection &bc, ConstraintCollection &cc, size_t i, scalar inverse_time_step);

/**
 * @brief Solves and applies a rotational constraint impulse between two bodies.
 *
 * - Computes generalized inverse masses for each body about the constraint axis.
 * - Computes Δλ and updates the constraint’s λ.
 * - Converts Δλ into an impulse and corresponding torque.
 * - Applies equal-and-opposite rotational impulses to each body.
 *
 * @param bc                The BodyCollection containing all bodies.
 * @param cc                The ConstraintCollection.
 * @param i                 Index of the constraint.
 * @param inverse_time_step Inverse of the time step (1/Δt).
 */
void compute_positional_constraint_impulse(BodyCollection &bc, ConstraintCollection &cc, size_t i, scalar inverse_time_step);


/**
 * @brief Resets the Lagrange multiplier for a given constraint to zero.
 *
 * @param cc The ConstraintCollection.
 * @param i  Index of the constraint to reset.
 */
void reset_lagrange_multiplier(ConstraintCollection &cc, size_t i);

/**
 * @brief Sets the local contact points for a constraint.
 *
 * @param cc  The ConstraintCollection.
 * @param i   Index of the constraint.
 * @param r_1 Local contact point on body 1.
 * @param r_2 Local contact point on body 2.
 */
void set_constraint_positions(ConstraintCollection &cc,size_t i, const vec3 &r_1, const vec3 &r_2);
/**
 * @brief Returns the current Lagrange multiplier λ for a constraint.
 *
 * @param cc The ConstraintCollection.
 * @param i  Index of the constraint.
 * @return   Current λ value.
 */
scalar get_lagrange_multiplier(const ConstraintCollection &cc, size_t i);

/**
 * @brief Computes the updated Lagrange multiplier for a single constraint.
 *
 * Does not store the result—returns λ_old + Δλ based on current state.
 *
 * @param bc                The BodyCollection.
 * @param cc                The ConstraintCollection.
 * @param i                 Index of the constraint.
 * @param inverse_time_step Inverse of the time step (1/Δt).
 * @return                  Updated Lagrange multiplier.
 */
scalar compute_lagrange_multiplier(BodyCollection &bc, ConstraintCollection &cc, size_t i, scalar inverse_time_step);