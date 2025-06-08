#include "physicsAcc/Constraints/ConstraintCollection.hpp"

void set_value(ConstraintCollection &cc, size_t i, vec3 value)
{
    cc.magnitude[i] = ti::magnitude(value);

    cc.direction[i] = (cc.magnitude[i] > EPSILON)
                          ? ti::normalize(value)
                          : vec3{1.0, 0.0, 0.0}; // Needs to be a unit vector
    
};

scalar compute_delta_lambda(const ConstraintCollection &cc, size_t i, scalar w_1, scalar w_2, scalar inverse_time_step)
{
    scalar alpha_p = cc.compliance[i] * inverse_time_step * inverse_time_step;
    return (-cc.magnitude[i] - alpha_p * cc.lambda[i]) / (w_1 + w_2 + alpha_p);
};

void compute_rotational_constraint_impulse(BodyCollection &bc, ConstraintCollection &cc, size_t i, scalar inverse_time_step)
{
    scalar w_1, w_2;
    w_1 = get_rotational_generalized_inverse_mass(bc, cc.body_1[i], cc.direction[i]);
    w_2 = get_rotational_generalized_inverse_mass(bc, cc.body_2[i], cc.direction[i]);
    scalar delta_lambda = compute_delta_lambda(cc, i, w_1, w_2, inverse_time_step);
    cc.lambda[i] += delta_lambda;
    cc.impulse[i] = delta_lambda * cc.direction[i];
    cc.torque[i] = cc.direction[i] * cc.lambda[i] * inverse_time_step * inverse_time_step;
    apply_rotational_constraint_impulse(bc, cc.body_1[i], cc.impulse[i]);
    apply_rotational_constraint_impulse(bc, cc.body_2[i], -cc.impulse[i]);
};

void compute_positional_constraint_impulse(BodyCollection &bc, ConstraintCollection &cc, size_t i, scalar inverse_time_step)
{
    vec3 r_1_wc = ti::rotate(bc.orientation[cc.body_1[i]], cc.r_1[i]);
    vec3 r_2_wc = ti::rotate(bc.orientation[cc.body_2[i]], cc.r_2[i]);
    scalar w_1, w_2;
    w_1 = get_positional_generalized_inverse_mass(bc, cc.body_1[i], r_1_wc, cc.direction[i]);
    w_2 = get_positional_generalized_inverse_mass(bc, cc.body_2[i], r_2_wc, cc.direction[i]);
    scalar delta_lambda = compute_delta_lambda(cc, i, w_1, w_2, inverse_time_step);
    cc.lambda[i] += delta_lambda;
    cc.impulse[i] = delta_lambda * cc.direction[i];
    cc.force[i] = cc.direction[i] * cc.lambda[i] * inverse_time_step * inverse_time_step;
    apply_positional_constraint_impulse(bc, cc.body_1[i], cc.impulse[i], r_1_wc);
    apply_positional_constraint_impulse(bc, cc.body_2[i], -cc.impulse[i], r_2_wc);
};

void reset_lagrange_multiplier(ConstraintCollection &cc, size_t i)
{
    cc.lambda[i] = 0.0;
};

void set_constraint_positions(ConstraintCollection &cc,
                              size_t i,
                              const vec3 &r_1,
                              const vec3 &r_2)
{
    cc.r_1[i] = r_1;
    cc.r_2[i] = r_2;
}

scalar get_lagrange_multiplier(const ConstraintCollection &cc, size_t i)
{
    return cc.lambda[i];
};

scalar compute_lagrange_multiplier(BodyCollection &bc, ConstraintCollection &cc, size_t i, scalar inverse_time_step)
{
    vec3 r_1_wc = ti::rotate(bc.orientation[cc.body_1[i]], cc.r_1[i]);
    vec3 r_2_wc = ti::rotate(bc.orientation[cc.body_2[i]], cc.r_2[i]);
    scalar w_1, w_2 = 0.0;
    switch (cc.type[i])
    {
    case ConstraintType::POSITIONAL:
        w_1 = get_positional_generalized_inverse_mass(bc, cc.body_1[i], r_1_wc, cc.direction[i]);
        w_2 = get_positional_generalized_inverse_mass(bc, cc.body_2[i], r_2_wc, cc.direction[i]);
        break;
    case ConstraintType::ROTATIONAL:
        w_1 = get_rotational_generalized_inverse_mass(bc, cc.body_1[i], cc.direction[i]);
        w_2 = get_rotational_generalized_inverse_mass(bc, cc.body_2[i], cc.direction[i]);
        break;
    default:
        break;
    }
    scalar delta_lambda = compute_delta_lambda(cc, i, w_1, w_2, inverse_time_step);
    return cc.lambda[i] + delta_lambda;
};

void solve_constraints(BodyCollection &bc, ConstraintCollection &cc, scalar inverse_time_step)
{

    for (size_t i = 0; i < cc.n_constraints; ++i)
    {   
        switch (cc.type[i])
        {
        case ConstraintType::POSITIONAL:
            compute_positional_constraint_impulse(bc, cc, i, inverse_time_step);
            break;
        case ConstraintType::ROTATIONAL:
            compute_rotational_constraint_impulse(bc, cc, i, inverse_time_step);
            break;
        default:
            break;
        }
    }
}