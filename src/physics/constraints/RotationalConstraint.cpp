#include "RotationalConstraint.hpp"

RotationalConstraint::RotationalConstraint(Body *body_1,
                                           Body *body_2,
                                           vec3 r_1,
                                           vec3 r_2,
                                           scalar compliance,
                                           scalar damping)
{
    // We asing the values of the initialization to the
    // Respective properties of the constraint class
    this->body_1 = body_1;
    this->body_2 = body_2;
    this->r_1 = r_1;
    this->r_2 = r_2;

    this->compliance = compliance;
    this->damping = damping;

    // The lagrange multiplier lambda is initialized as zero
    this->lambda = 0;
    // We also initialize the force of the constraint as a
    // zero vector
    this->torque = vec3(0.0, 0.0, 0.0);
}

void RotationalConstraint::set_value(vec3 value)
{
    this->magnitude = ti::magnitude(value);

    // Check if the magnitude is near an epsilon
    if (magnitude > EPSILON)
    {
        this->n = ti::normalize(value);
    }
    else
    {
        this->n = vec3(1.0, 0.0, 0.0);
    }
}

scalar RotationalConstraint::compute_rotational_delta_lambda(scalar w_1, scalar w_2, scalar inverse_time_step)
{

    scalar alpha_p = this->compliance * inverse_time_step * inverse_time_step;

    return (-this->magnitude - alpha_p * this->lambda) / (w_1 + w_2 + alpha_p);
}

void RotationalConstraint::apply_constraint(scalar inverse_time_step)
{

    // Calculte the generalized inverse mass of the bodies
    scalar w_1 = this->body_1->get_rotational_generalized_inverse_mass(this->n);
    scalar w_2 = this->body_2->get_rotational_generalized_inverse_mass(this->n);

    // Calculate the change in the lagrange multiplier (delta lambda)
    scalar delta_lambda = this->compute_rotational_delta_lambda(w_1, w_2, inverse_time_step);
    
    // Update the lagrange multiplier
    this->lambda += delta_lambda;

    // Compute the impulse and force of the constraint
    vec3 impulse = delta_lambda * this->n;
    this->torque = this->n * this->lambda * inverse_time_step * inverse_time_step;

    // Apply the impulse to both bodies
    this->body_1->apply_rotational_constraint_impulse(impulse);
    this->body_2->apply_rotational_constraint_impulse(-impulse); // Note the negative sign at the impulse!
}
