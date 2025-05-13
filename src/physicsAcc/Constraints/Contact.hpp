#pragma once
#include <vector>
#include "physics/math/math.hpp"
#include "physicsAcc/Constraints/ConstraintCollection.hpp"
#include "physicsAcc/Body/BodyCollection.hpp"

/**
 * @brief Collection of per-contact data for non-persistent collision handling.
 *
 * Each contact represents a potential collision between two bodies and stores
 * all information needed to resolve the normal and friction constraints at the
 * position- and velocity-levels.
 */
struct ContactCollection
{
    size_t n_contacts = 0; // Active contacts
    std::vector<size_t> body_1 = {};
    std::vector<size_t> body_2 = {};
    std::vector<vec3> normal = {};
    std::vector<vec3> p_1 = {};
    std::vector<vec3> p_2 = {};
    std::vector<bool> collision = {};
    std::vector<scalar> static_friction = {};  // Static friction of the contact
    std::vector<scalar> dynamic_friction = {}; // Dynamic friction of the contact
    std::vector<scalar> restitution = {};      // Restitution coefficient of the contact
    std::vector<scalar> relative_velocity = {};
    std::vector<scalar> normal_constraint_lagrange_multiplier = {};
    std::vector<scalar> tangencial_constraint_lagrange_multiplier = {};
    std::vector<vec3> normal_force = {};
    std::vector<vec3> tangencial_force = {};
};

/**
 * @brief Resolves a single contact at the position level.
 *
 * Checks penetration depth along the contact normal and, if positive:
 * 1. Marks collision[i] = true.
 * 2. Solves the normal constraint (non-penetration) via solve_normal_constraint().
 * 3. Computes pre-solver normal relative velocity.
 * 4. Solves the tangential (friction) constraint via solve_tangencial_constraint().
 *
 * @param cc                 The ContactCollection.
 * @param i                  Index of the contact to resolve.
 * @param bc                 The BodyCollection holding current body states.
 * @param inverse_time_step  Inverse of the timestep (1/Δt).
 */
void apply_constraint_position_level(ContactCollection &cc, size_t i, BodyCollection &bc, scalar inverse_time_step);
/**
 * @brief Solves the non-penetration (normal) constraint for one contact.
 *
 * Computes Δλ for the normal constraint:
 *   Δλ = (–penetration_depth – α·λ_old) / (w₁ + w₂ + α),
 * where w₁, w₂ are the generalized inverse masses along the contact normal.
 * Applies the resulting impulse to both bodies and updates normal_force and λ.
 *
 * @param cc                 The ContactCollection.
 * @param i                  Index of the contact.
 * @param bc                 The BodyCollection.
 * @param inverse_time_step  Inverse of the timestep (1/Δt).
 * @param magnitude          Penetration depth (positive if bodies overlap).
 * @param r_1_wc             World-space lever arm from body_1 COM to contact.
 * @param r_2_wc             World-space lever arm from body_2 COM to contact.
 */
void solve_normal_constraint(ContactCollection &cc,
                             size_t i,
                             BodyCollection &bc,
                             scalar inverse_time_step,
                             scalar magnitude,
                             vec3 r_1_wc,
                             vec3 r_2_wc);
/**
 * @brief Solves the tangential (friction) constraint for one contact.
 *
 * 1. Computes tangential slip Δp_t = Δp – (Δp·n)n from the change in contact points.
 * 2. Computes Δλ for the friction constraint similarly to the normal case.
 * 3. Applies the friction impulse only if |λ_t| ≤ μ_s·λ_n (static friction)
 *    or else applies dynamic friction.
 * 4. Updates tangencial_force and λ_t.
 *
 * @param cc                 The ContactCollection.
 * @param i                  Index of the contact.
 * @param bc                 The BodyCollection.
 * @param inverse_time_step  Inverse of the timestep (1/Δt).
 */
void solve_tangencial_constraint(ContactCollection &cc, size_t i, BodyCollection &bc, scalar inverse_time_step);

/**
 * @brief Resolves a single contact at the velocity level (restitution & friction).
 *
 * Checks for active collision and then:
 * 1. Computes relative velocity at the contact point.
 * 2. Applies restitution along the normal direction if bodies are separating.
 * 3. Computes and applies Coulomb friction impulses (dynamic) along the tangential plane.
 * 4. Uses apply_positional_velocity_constraint_impulse() to adjust body velocities.
 *
 * @param cc         The ContactCollection.
 * @param i          Index of the contact to resolve.
 * @param bc         The BodyCollection holding body velocities.
 * @param time_step  Simulation timestep Δt (used for restitution thresholding).
 */
void apply_constraint_velocity_level(ContactCollection &cc, size_t i, BodyCollection &bc, scalar time_step);

/**
 * @brief Applies the constraints on all contacts at the velocity level
 */

void solve_contacts_velocity_level(ContactCollection &cc, BodyCollection &bc, scalar time_step);