#include "RevoluteJointConstraint.hpp"

RevoluteJointConstraint::RevoluteJointConstraint(Body *body_1,
                                                 Body *body_2,
                                                 vec3 aligned_axis,
                                                 vec3 r_1,
                                                 vec3 r_2,
                                                 scalar compliance,
                                                 scalar damping,
                                                 RevoluteJointType type,
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
    this->attachment_point_constraint = new PositionalConstraint(body_1, body_2, r_1, r_2, 0.0, 0.0);
    this->drive_joint_constraint = new RotationalConstraint(body_1, body_2, r_1, r_2, compliance, damping);

    this->angle_limit_constraint = new RotationalConstraint(body_1, body_2, r_1, r_2, 0.0, 0.0);
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

    // Apply the constraints
    this->attachment_point_constraint->set_value(p_1 - p_2);
    this->attachment_point_constraint->apply_constraint(inverse_time_step);

    a_1 = ti::rotate(this->body_1->orientation, this->aligned_axis ) ;
    vec3 secondary_axis = {this->aligned_axis.z, this->aligned_axis.x, this->aligned_axis.y};
    vec3 b_1 = ti::rotate(this->body_1->orientation, secondary_axis ) ;//body_1_ax[1];
    vec3 b_2 = ti::rotate(this->body_2->orientation, secondary_axis ) ;//body_2_ax[1];
    

    // Calculate the joint value:
    this->current_angle = ti::atan2(ti::dot(ti::cross(b_1, b_2), a_1), ti::dot(b_1, b_2));

    // Calculate the constraint if nessesary
    if (this->limited)
    {
        auto response = compute_angle_limit_constraint_value(this->current_angle, a_1, b_1, b_2, this->lower_limit, this->upper_limit);

        this->current_angle = response.angle;

        this->angle_limit_constraint->set_value(-response.delta_q);

        this->angle_limit_constraint->apply_constraint(inverse_time_step);
    }

    if (this->type != FREE)
    {
        if (this->type == DRIVEN_BY_SPEED){
            this->target_angle = this->current_angle + this->target_speed / inverse_time_step;
        }
        if(this->limited){// Recalculate the axes if a previous correction was applied
            a_1 = ti::rotate(this->body_1->orientation, this->aligned_axis ) ;
            b_1 = ti::rotate(this->body_1->orientation, secondary_axis);
            b_2 = ti::rotate(this->body_2->orientation, secondary_axis);
            this->current_angle = ti::atan2(ti::dot(ti::cross(b_1, b_2), a_1), ti::dot(b_1, b_2));
        }
            
        vec3 b_target = b_1 * ti::cos(this->target_angle) +
                        ti::cross(a_1, b_1) * ti::sin(this->target_angle) +
                        a_1 * ti::dot(a_1, b_1) * (1 - ti::cos(this->target_angle));

        vec3 delta_q_target = ti::cross(b_target, b_2);

        this->drive_joint_constraint->set_value(delta_q_target);

        this->drive_joint_constraint->apply_constraint(inverse_time_step);
    }
}

void RevoluteJointConstraint::apply_joint_damping(scalar time_step){
    
    vec3 delta_omega = (this->body_2->angular_velocity - this->body_1->angular_velocity) * std::min(this->damping * time_step, 1.0);

    this->body_1->angular_velocity += delta_omega;
    this->body_2->angular_velocity -= delta_omega;
    
}