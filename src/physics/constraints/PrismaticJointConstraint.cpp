#include "PrismaticJoinConstraint.hpp"

PrismaticJointConstraint::PrismaticJointConstraint(std::shared_ptr<Body> body_1,
                                                 std::shared_ptr<Body> body_2,
                                                 vec3 aligned_axis,
                                                 vec3 r_1,
                                                 vec3 r_2,
                                                 scalar compliance,
                                                 scalar damping,
                                                 PrismaticJointType type,
                                                 bool limited,
                                                 scalar lower_limit,
                                                 scalar upper_limit)
{
    this->body_1 = body_1;
    this->body_2 = body_2;
    this->aligned_axis = aligned_axis;

    this->r_1 = r_1;
    this->r_2 = r_2;

    this->limited = limited;
    this->lower_limit = lower_limit;
    this->upper_limit = upper_limit;

    this->compliance = compliance;
    this->damping = damping;

    this->type = type;

    // Initialize the contsraints
    this->offset_constraint = new PositionalConstraint(body_1, body_2, r_1, r_2, compliance, damping);
    this->aligned_constraint = new RotationalConstraint(body_1, body_2, r_1, r_2, 0.0, 0.0);
}


void RevoluteJointConstraint::apply_constraint(scalar inverse_time_step)
{

    // Get the axes:
    vec3 a_1 = ti::rotate(this->body_1->orientation, this->aligned_axis );
    vec3 a_2 = ti::rotate(this->body_2->orientation, this->aligned_axis );

    vec3 delta_q = - ti::cross(a_1, a_2); 

    // Aligned constraint (It is of upmost importance to apply the constraints as soon as possible !!!)
    this->aligned_constraint->set_value(delta_q);
    this->aligned_constraint->apply_constraint(inverse_time_step);

    // Get the world coordinates of r vectors of the two constriant bodies
    vec3 r_1_wc = ti::rotate(this->body_1->orientation, this->r_1);
    vec3 r_2_wc = ti::rotate(this->body_2->orientation, this->r_2);

    // Attachement point constraint
    vec3 p_1 = this->body_1->position + r_1_wc;
    vec3 p_2 = this->body_2->position + r_2_wc;

    vec3 delta_r =  p_2 - p_1;

    // Apply the constraints
    this->offset_constraint->set_value(p_1 - p_2);
    this->offset_constraint->apply_constraint(inverse_time_step);

}
