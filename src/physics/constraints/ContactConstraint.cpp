#include "ContactConstraint.hpp"

ContactConstraint::ContactConstraint(Body *body_1,
                                     Body *body_2,
                                     vec3 normal,
                                     vec3 p_1,
                                     vec3 p_2,
                                     scalar static_fricction,
                                     scalar dynamic_fricction,
                                     scalar restitution)
{
    this->body_1 = body_1;
    this->body_2 = body_2;

    this->normal = normal;

    this->p_1 = p_1;
    this->p_2 = p_2;

    this->static_fricction = static_fricction;
    this->dynamic_fricction = dynamic_fricction;
    this->restitution = restitution;
}

void ContactConstraint::apply_constraint_position_level(scalar inverse_time_step)
{

    vec3 n = this->normal;
    vec3 p_1 = this->p_1;
    vec3 p_2 = this->p_2;
    scalar d = ti::dot((p_1 - p_2), n);
    if (d <= 0)
    {
        this->collision = false;
        return;
    }

    this->collision = true;

    vec3 r_1_wc = p_1 - this->body_1->position;
    vec3 r_2_wc = p_2 - this->body_2->position;

    vec3 r_1 = ti::rotate(ti::inverse(this->body_1->orientation), r_1_wc);
    vec3 r_2 = ti::rotate(ti::inverse(this->body_2->orientation), r_2_wc);

    this->solve_normal_constraint(inverse_time_step, d);

    // We calculate the relative velocity:
    vec3 v_n = (this->body_1->linear_velocity + ti::cross(this->body_1->angular_velocity, r_1_wc)) -
               (this->body_2->linear_velocity + ti::cross(this->body_2->angular_velocity, r_2_wc));

    this->relative_velocity = ti::dot(v_n, n);

    this->solve_tangencial_constraint(inverse_time_step);
}

void ContactConstraint::apply_constraint_velocity_level(scalar time_step)
{

    vec3 n = this->normal;
    vec3 p_1 = this->p_1;
    vec3 p_2 = this->p_2;
    scalar d = ti::dot((p_1 - p_2), n);

    if (d <= 0)
    {
        this->collision = false;
        return;
    }

    vec3 delta_v = {0.0, 0.0, 0.0};

    vec3 r_1_wc = p_1 - this->body_1->position;
    vec3 r_2_wc = p_2 - this->body_2->position;

    // Recalculate the new velocity
    vec3 v = (this->body_1->linear_velocity + ti::cross(this->body_1->angular_velocity, r_1_wc)) -
             (this->body_2->linear_velocity + ti::cross(this->body_2->angular_velocity, r_2_wc));

    scalar v_n = ti::dot(v, n);
    vec3 v_t = v - n * v_n;

    if (ti::magnitude(v_t) > EPSILON)
    {
        scalar friction = ti::min(-this->dynamic_fricction * this->normal_constraint_lagrange_multiplier / time_step,
                                  ti::magnitude(v_t));

        delta_v += -ti::normalize(v_t) * friction;
    }

    scalar contact_restitution = this->restitution;
    if (ti::abs(v_n) <= 2.0 * 9.8 * time_step)
    {
        contact_restitution = 0.0;
    }

    delta_v += n * (-v_n + ti::min(-contact_restitution * this->relative_velocity, 0.0));

    scalar w_1 = this->body_1->get_positional_generalized_inverse_mass(r_1_wc, n);
    scalar w_2 = this->body_2->get_positional_generalized_inverse_mass(r_2_wc, n);

    vec3 impulse = delta_v / (w_1 + w_2);

    this->body_1->apply_positional_velocity_constraint_impulse(impulse, r_1_wc);
    this->body_2->apply_positional_velocity_constraint_impulse(-impulse, r_2_wc);
}

void ContactConstraint::solve_normal_constraint(scalar inverse_time_step, scalar magnitude)
{

    vec3 r_1_wc = p_1 - this->body_1->position;
    vec3 r_2_wc = p_2 - this->body_2->position;

    scalar w_1 = this->body_1->get_positional_generalized_inverse_mass(r_1_wc, this->normal);
    scalar w_2 = this->body_2->get_positional_generalized_inverse_mass(r_2_wc, this->normal);

    // Calculate the change in the lagrange multiplier (delta lambda)

    // I left this because it might be useful for soft contacts, but for regular contacts it should be 0
    scalar alpha_p = 0.0; // this->compliance * inverse_time_step * inverse_time_step;

    scalar delta_lambda = (-magnitude - alpha_p * this->normal_constraint_lagrange_multiplier) / (w_1 + w_2 + alpha_p);

    // Update the lagrange multiplier
    this->normal_constraint_lagrange_multiplier += delta_lambda;

    // Compute the impulse and force of the constraint
    vec3 impulse = delta_lambda * this->normal;
    this->normal_force = this->normal * this->normal_constraint_lagrange_multiplier * inverse_time_step * inverse_time_step;

    // Apply the impulse to both bodies
    this->body_1->apply_positional_constraint_impulse(impulse, r_1_wc);
    this->body_2->apply_positional_constraint_impulse(-impulse, r_2_wc); // Note the negative sign at the impulse!
}

void ContactConstraint::solve_tangencial_constraint(scalar inverse_time_step)
{

    // Note this are recalculated for the previous positions
    // Double check this next 4 lines (Here we may have to use the current and not the previous position)
    vec3 r_1_wc = p_1 - this->body_1->position;
    vec3 r_2_wc = p_2 - this->body_2->position;
    vec3 r_1 = ti::rotate(ti::inverse(this->body_1->orientation), r_1_wc);
    vec3 r_2 = ti::rotate(ti::inverse(this->body_2->orientation), r_2_wc);

    // This part is Ok
    vec3 prev_p_1 = this->body_1->prev_position + ti::rotate(this->body_1->prev_orientation, r_1);
    vec3 prev_p_2 = this->body_2->prev_position + ti::rotate(this->body_2->prev_orientation, r_2);

    vec3 delta_p = (p_1 - prev_p_1) - (p_2 - prev_p_2);

    vec3 delta_p_tangencial = delta_p - (ti::dot(delta_p, this->normal)) * this->normal;

    // Contraint magnitude
    scalar magnitude = ti::magnitude(delta_p_tangencial);

    if (magnitude < EPSILON)
    {
        return;
    }

    // Constraint direction
    vec3 n = ti::normalize(delta_p_tangencial);

    // Calculte the generalized inverse mass of the bodies
    scalar w_1 = this->body_1->get_positional_generalized_inverse_mass(r_1_wc, n);
    scalar w_2 = this->body_2->get_positional_generalized_inverse_mass(r_2_wc, n);

    scalar alpha_p = 0.0; // this->compliance * inverse_time_step * inverse_time_step;

    scalar delta_lambda = (-magnitude - alpha_p * this->tangencial_constraint_lagrange_multiplier) / (w_1 + w_2 + alpha_p);

    // CHECK:  We might have to update it if the constraint is applied (I am not sure)
    this->tangencial_constraint_lagrange_multiplier += delta_lambda;

    // Solve for tangencial constraint
    scalar lambda_t = this->tangencial_constraint_lagrange_multiplier;
    scalar lambda_n = this->normal_constraint_lagrange_multiplier;

    // Only apply the if the condition is satisfied
    if (lambda_t > lambda_n * this->static_fricction)
    {
        vec3 impulse = delta_lambda * n;
        this->tangencial_force = n * this->tangencial_constraint_lagrange_multiplier * inverse_time_step * inverse_time_step;

        // Apply the impulse to both bodies
        this->body_1->apply_positional_constraint_impulse(impulse, r_1_wc);
        this->body_2->apply_positional_constraint_impulse(-impulse, r_2_wc); // Note the negative sign at the impulse!
    }
}