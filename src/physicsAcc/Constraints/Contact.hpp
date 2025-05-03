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
    size_t n_contacts; // Active contacts
    std::vector<size_t> body_1;
    std::vector<size_t> body_2;
    std::vector<vec3> normal;
    std::vector<vec3> p_1;
    std::vector<vec3> p_2;
    std::vector<bool> collision;
    std::vector<scalar> static_fricction;  // Static fricction of the contact
    std::vector<scalar> dynamic_fricction; // Dynamic friction of the contact
    std::vector<scalar> restitution;       // Restitution coefficient of the contact
    std::vector<scalar> relative_velocity;
    std::vector<scalar> normal_constraint_lagrange_multiplier;
    std::vector<scalar> tangencial_constraint_lagrange_multiplier;
    std::vector<vec3> normal_force;
    std::vector<vec3> tangencial_force;
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
void apply_constraint_position_level(ContactCollection &cc, size_t i, BodyCollection &bc, scalar inverse_time_step)
{

    scalar d = ti::dot((cc.p_1[i] - cc.p_2[i]), cc.normal[i]);
    if (d <= 0)
    {
        cc.collision[i] = false;
        return;
    }

    cc.collision[i] = true;

    vec3 r_1_wc = cc.p_1[i] - bc.position[cc.body_1[i]];
    vec3 r_2_wc = cc.p_2[i] - bc.position[cc.body_2[i]];

    vec3 r_1 = ti::rotate(ti::inverse(bc.orientation[cc.body_1[i]]), r_1_wc);
    vec3 r_2 = ti::rotate(ti::inverse(bc.orientation[cc.body_2[i]]), r_2_wc);

    solve_normal_constraint(cc, i, bc, inverse_time_step, d, r_1_wc, r_2_wc);

    // We calculate the relative velocity:
    vec3 v_n = (bc.linear_velocity[cc.body_1[i]] + ti::cross(bc.angular_velocity[cc.body_1[i]], r_1_wc)) -
               (bc.linear_velocity[cc.body_2[i]] + ti::cross(bc.angular_velocity[cc.body_2[i]], r_2_wc));

    cc.relative_velocity[i] = ti::dot(v_n, cc.normal[i]);

    solve_tangencial_constraint(cc, i, bc, inverse_time_step);
}

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
                             vec3 r_2_wc)
{

    scalar w_1 = get_positional_generalized_inverse_mass(bc, cc.body_1[i], r_1_wc, cc.normal[i]);
    scalar w_2 = get_positional_generalized_inverse_mass(bc, cc.body_2[i], r_2_wc, cc.normal[i]);

    // Calculate the change in the lagrange multiplier (delta lambda)

    // I left this because it might be useful for soft contacts, but for regular contacts it should be 0
    scalar alpha_p = 0.0; // this->compliance * inverse_time_step * inverse_time_step;

    scalar delta_lambda = (-magnitude - alpha_p * cc.normal_constraint_lagrange_multiplier[i]) / (w_1 + w_2 + alpha_p);

    // Update the lagrange multiplier
    cc.normal_constraint_lagrange_multiplier[i] += delta_lambda;

    // Compute the impulse and force of the constraint
    vec3 impulse = delta_lambda * cc.normal[i];
    cc.normal_force[i] = cc.normal[i] * cc.normal_constraint_lagrange_multiplier[i] * inverse_time_step * inverse_time_step;

    // Apply the impulse to both bodies
    apply_positional_constraint_impulse(bc, cc.body_1[i], impulse, r_1_wc);
    apply_positional_constraint_impulse(bc, cc.body_2[i], -impulse, r_2_wc);
}

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
void solve_tangencial_constraint(ContactCollection &cc, size_t i, BodyCollection &bc, scalar inverse_time_step)
{

    // Note this are recalculated for the previous positions
    // Double check this next 4 lines (Here we may have to use the current and not the previous position)
    vec3 r_1_wc = cc.p_1[i] - bc.position[cc.body_1[i]];
    vec3 r_2_wc = cc.p_2[i] - bc.position[cc.body_2[i]];
    vec3 r_1 = ti::rotate(ti::inverse(bc.orientation[cc.body_1[i]]), r_1_wc);
    vec3 r_2 = ti::rotate(ti::inverse(bc.orientation[cc.body_2[i]]), r_2_wc);

    // This part is Ok
    vec3 prev_p_1 = bc.prev_position[cc.body_1[i]] + ti::rotate(bc.prev_orientation[cc.body_1[i]], r_1);
    vec3 prev_p_2 = bc.prev_position[cc.body_2[i]] + ti::rotate(bc.prev_orientation[cc.body_2[i]], r_2);

    vec3 delta_p = (cc.p_1[i] - prev_p_1) - (cc.p_2[i] - prev_p_2);

    vec3 delta_p_tangencial = delta_p - (ti::dot(delta_p, cc.normal[i])) * cc.normal[i];

    // Contraint magnitude
    scalar magnitude = ti::magnitude(delta_p_tangencial);

    if (magnitude < EPSILON)
    {
        return;
    }

    // Constraint direction
    vec3 n = ti::normalize(delta_p_tangencial);

    // Calculte the generalized inverse mass of the bodies
    scalar w_1 = get_positional_generalized_inverse_mass(bc, cc.body_1[i], r_1_wc, n);
    scalar w_2 = get_positional_generalized_inverse_mass(bc, cc.body_2[i], r_2_wc, n);

    scalar alpha_p = 0.0; // this->compliance * inverse_time_step * inverse_time_step;

    scalar delta_lambda = (-magnitude - alpha_p * cc.tangencial_constraint_lagrange_multiplier[i]) / (w_1 + w_2 + alpha_p);

    // CHECK:  We might have to update it if the constraint is applied (I am not sure)
    cc.tangencial_constraint_lagrange_multiplier[i] += delta_lambda;

    // Solve for tangencial constraint
    scalar lambda_t = cc.tangencial_constraint_lagrange_multiplier[i];
    scalar lambda_n = cc.normal_constraint_lagrange_multiplier[i];

    // Only apply the if the condition is satisfied
    if (lambda_t > lambda_n * cc.static_fricction[i])
    {
        vec3 impulse = delta_lambda * n;
        cc.tangencial_force[i] = n * cc.tangencial_constraint_lagrange_multiplier[i] * inverse_time_step * inverse_time_step;

        // Apply the impulse to both bodies
        apply_positional_constraint_impulse(bc, cc.body_1[i], impulse, r_1_wc);
        apply_positional_constraint_impulse(bc, cc.body_2[i], -impulse, r_2_wc);
    }
}

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
void apply_constraint_velocity_level(ContactCollection &cc, size_t i, BodyCollection &bc, scalar time_step)
{

    scalar d = ti::dot((cc.p_1[i] - cc.p_2[i]), cc.normal[i]);
    if (d <= 0)
    {
        cc.collision[i] = false;
        return;
    }
    cc.collision[i] = true;
    vec3 delta_v = {0.0, 0.0, 0.0};

    vec3 r_1_wc = cc.p_1[i] - bc.position[cc.body_1[i]];
    vec3 r_2_wc = cc.p_2[i] - bc.position[cc.body_2[i]];

    // Recalculate the new velocity
    vec3 v = (bc.linear_velocity[cc.body_1[i]] + ti::cross(bc.angular_velocity[cc.body_1[i]], r_1_wc)) -
             (bc.linear_velocity[cc.body_2[i]] + ti::cross(bc.angular_velocity[cc.body_2[i]], r_2_wc));

    scalar v_n = ti::dot(v, cc.normal[i]);
    vec3 v_t = v - cc.normal[i] * v_n;

    if (ti::magnitude(v_t) > EPSILON)
    {
        scalar friction = ti::min(-cc.dynamic_fricction[i] * cc.normal_constraint_lagrange_multiplier[i] / time_step,
                                  ti::magnitude(v_t));

        delta_v += -ti::normalize(v_t) * friction;
    }

    scalar contact_restitution = cc.restitution[i];
    if (ti::abs(v_n) <= 2.0 * 9.8 * time_step)
    {
        contact_restitution = 0.0;
    }

    delta_v += cc.normal[i] * (-v_n + ti::min(-contact_restitution * cc.relative_velocity[i], 0.0));

    scalar w_1 = get_positional_generalized_inverse_mass(bc, cc.body_1[i], r_1_wc, cc.normal[i]);
    scalar w_2 = get_positional_generalized_inverse_mass(bc, cc.body_2[i], r_2_wc, cc.normal[i]);

    vec3 impulse = delta_v / (w_1 + w_2);

    apply_positional_velocity_constraint_impulse(bc, cc.body_1[i], impulse, r_1_wc);
    apply_positional_velocity_constraint_impulse(bc, cc.body_2[i], -impulse, r_2_wc);
}
