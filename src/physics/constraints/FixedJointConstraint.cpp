#include "FixedJointConstraint.hpp"

FixedJointConstraint::FixedJointConstraint(std::shared_ptr<Body> body_1,
                                            std::shared_ptr<Body> body_2,
                                            vec3 r_1,
                                            vec3 r_2)
{
    this->body_1 = body_1;
    this->body_2 = body_2;
    this->r_1 = r_1;
    this->r_2 = r_2;

    // Initialize the constraints
    this->attachment_point_constraint = std::make_shared<PositionalConstraint>(body_1, body_2, r_1, r_2, 0.0, 0.0);
    this->aligned_constraint = std::make_shared<RotationalConstraint>(body_1, body_2, r_1, r_2, 0.0, 0.0);
}

void FixedJointConstraint::apply_constraint(scalar inverse_time_step, scalar time_step)
{

    quat dq = this->body_1->orientation * ti::conjugate(this->body_2->orientation);
    vec3 delta_q = 2.0 * vec3{dq.x, dq.y, dq.z};

    // Aligned constraint (It is of upmost importance to apply the constraints as soon as possible !!!)
    this->aligned_constraint->set_value(delta_q);
    this->aligned_constraint->apply_constraint(inverse_time_step);

    // Get the world coordinates of r vectors of the two constriant bodies
    vec3 r_1_wc = ti::rotate(this->body_1->orientation, this->r_1);
    vec3 r_2_wc = ti::rotate(this->body_2->orientation, this->r_2);

    vec3 p_1 = this->body_1->position + r_1_wc;
    vec3 p_2 = this->body_2->position + r_2_wc;
    vec3 delta_p = p_1 - p_2;
    
    vec3 correction  = delta_p ;

    // // Apply the constraints
    this->attachment_point_constraint->set_value(correction);
    this->attachment_point_constraint->apply_constraint(inverse_time_step);    
}


void FixedJointConstraint::reset_lagrange_multipliers(){
    this->attachment_point_constraint->reset_lagrange_multiplier();
    this->aligned_constraint->reset_lagrange_multiplier();
}

FixedJointInfo FixedJointConstraint::get_info(void){
    FixedJointInfo info= FixedJointInfo{
        .position = this->body_1->position + ti::rotate(this->body_1->orientation, this->r_1),
        .force = this->force,
        .torque = this->torque,
        };

    return info;
}
