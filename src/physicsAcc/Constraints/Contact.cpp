#include "physicsAcc/Constraints/Contact.hpp"

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
    vec3 v_n = (bc.linear_velocity[cc.body_1[i]] + ti::cross(bc.angular_velocity[cc.body_1[i]], r_1_wc)) -
               (bc.linear_velocity[cc.body_2[i]] + ti::cross(bc.angular_velocity[cc.body_2[i]], r_2_wc));
    cc.relative_velocity[i] = ti::dot(v_n, cc.normal[i]);
    solve_tangencial_constraint(cc, i, bc, inverse_time_step);
}

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
    // I left this because it might be useful for soft contacts, but for regular contacts it should be 0
    scalar alpha_p = 0.0; // this->compliance * inverse_time_step * inverse_time_step;
    scalar delta_lambda = (-magnitude - alpha_p * cc.normal_constraint_lagrange_multiplier[i]) / (w_1 + w_2 + alpha_p);
    cc.normal_constraint_lagrange_multiplier[i] += delta_lambda;
    vec3 impulse = delta_lambda * cc.normal[i];
    cc.normal_force[i] = cc.normal[i] * cc.normal_constraint_lagrange_multiplier[i] * inverse_time_step * inverse_time_step;
    apply_positional_constraint_impulse(bc, cc.body_1[i], impulse, r_1_wc);
    apply_positional_constraint_impulse(bc, cc.body_2[i], -impulse, r_2_wc);
}

void solve_tangencial_constraint(ContactCollection &cc, size_t i, BodyCollection &bc, scalar inverse_time_step)
{
    // Note this are recalculated for the previous positions
    // Double check this next 4 lines (Here we may have to use the current and not the previous position)
    vec3 r_1_wc = cc.p_1[i] - bc.position[cc.body_1[i]];
    vec3 r_2_wc = cc.p_2[i] - bc.position[cc.body_2[i]];
    vec3 r_1 = ti::rotate(ti::inverse(bc.orientation[cc.body_1[i]]), r_1_wc);
    vec3 r_2 = ti::rotate(ti::inverse(bc.orientation[cc.body_2[i]]), r_2_wc);
    vec3 prev_p_1 = bc.prev_position[cc.body_1[i]] + ti::rotate(bc.prev_orientation[cc.body_1[i]], r_1);
    vec3 prev_p_2 = bc.prev_position[cc.body_2[i]] + ti::rotate(bc.prev_orientation[cc.body_2[i]], r_2);
    vec3 delta_p = (cc.p_1[i] - prev_p_1) - (cc.p_2[i] - prev_p_2);
    vec3 delta_p_tangencial = delta_p - (ti::dot(delta_p, cc.normal[i])) * cc.normal[i];
    scalar magnitude = ti::magnitude(delta_p_tangencial);
    if (magnitude < EPSILON)
    {
        return;
    }
    vec3 n = ti::normalize(delta_p_tangencial);
    scalar w_1 = get_positional_generalized_inverse_mass(bc, cc.body_1[i], r_1_wc, n);
    scalar w_2 = get_positional_generalized_inverse_mass(bc, cc.body_2[i], r_2_wc, n);
    scalar alpha_p = 0.0; // this->compliance * inverse_time_step * inverse_time_step;
    scalar delta_lambda = (-magnitude - alpha_p * cc.tangencial_constraint_lagrange_multiplier[i]) / (w_1 + w_2 + alpha_p);
    // CHECK:  We might have to update only if the constraint is applied (I am not sure)
    cc.tangencial_constraint_lagrange_multiplier[i] += delta_lambda;
    // Solve for tangencial constraint
    scalar lambda_t = cc.tangencial_constraint_lagrange_multiplier[i];
    scalar lambda_n = cc.normal_constraint_lagrange_multiplier[i];
    // Only apply the if the condition is satisfied
    if (lambda_t > lambda_n * cc.static_friction[i])
    {
        vec3 impulse = delta_lambda * n;
        cc.tangencial_force[i] = n * cc.tangencial_constraint_lagrange_multiplier[i] * inverse_time_step * inverse_time_step;
        // Apply the impulse to both bodies
        apply_positional_constraint_impulse(bc, cc.body_1[i], impulse, r_1_wc);
        apply_positional_constraint_impulse(bc, cc.body_2[i], -impulse, r_2_wc);
    }
}

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
        scalar friction = ti::min(-cc.dynamic_friction[i] * cc.normal_constraint_lagrange_multiplier[i] / time_step,
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
