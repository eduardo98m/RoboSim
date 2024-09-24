#include "PrismaticJointConstraint.hpp"

PrismaticJointConstraint::PrismaticJointConstraint(std::shared_ptr<Body> body_1,
                                                 std::shared_ptr<Body> body_2,
                                                 vec3 moving_axis,
                                                 vec3 r_1,
                                                 vec3 r_2,
                                                 scalar compliance,
                                                 scalar damping,
                                                 JointControlType type,
                                                 bool limited,
                                                 scalar lower_limit,
                                                 scalar upper_limit)
{
    this->body_1 = body_1;
    this->body_2 = body_2;
    this->moving_axis = moving_axis;
    this->r_1 = r_1;
    this->r_2 = r_2;

    this->limited = limited;
    this->lower_limit = lower_limit;
    this->upper_limit = upper_limit;

    this->compliance = compliance;
    this->damping = damping;

    this->type = type;

    // Initialize the contsraints
    this->attachment_point_constraint = std::make_shared<PositionalConstraint>(body_1, body_2, r_1, r_2, 0.0, 0.0);
    this->drive_joint_constraint = std::make_shared<PositionalConstraint>(body_1, body_2, r_1, r_2, compliance, damping);
    this->aligned_constraint = std::make_shared<RotationalConstraint>(body_1, body_2, r_1, r_2, 0.0, 0.0);
}

void PrismaticJointConstraint::apply_constraint(scalar inverse_time_step, scalar time_step)
{

    quat dq = this->body_1->orientation * ti::conjugate(this->body_2->orientation);
    vec3 delta_q = 2.0 * vec3{dq.x, dq.y, dq.z};

    // Aligned constraint (It is of upmost importance to apply the constraints as soon as possible !!!)
    this->aligned_constraint->set_value(delta_q);
    this->aligned_constraint->apply_constraint(inverse_time_step);

    // Get the world coordinates of r vectors of the two constriant bodies
    vec3 r_1_wc = ti::rotate(this->body_1->orientation, this->r_1);
    vec3 r_2_wc = ti::rotate(this->body_2->orientation, this->r_2);

    // We convert the moving (sliding) axis into world coordinates 
    vec3 moving_axis_wc = ti::rotate(this->body_1->orientation, this->moving_axis);


    vec3 p_1 = this->body_1->position + r_1_wc;
    vec3 p_2 = this->body_2->position + r_2_wc;
    vec3 delta_p = p_1 - p_2;
    this->current_position = ti::dot(delta_p, moving_axis_wc);
    scalar overshoot = 0.0;
    if (this->limited){
        overshoot =  this->current_position - ti::clamp(this->current_position, this->lower_limit, this->upper_limit);
    }

    
    vec3 correction  = delta_p - moving_axis_wc * this->current_position  + moving_axis_wc * overshoot;

    // // Apply the constraints
    this->attachment_point_constraint->set_value(correction);
    this->attachment_point_constraint->apply_constraint(inverse_time_step);


    r_1_wc = ti::rotate(this->body_1->orientation, this->r_1);
    r_2_wc = ti::rotate(this->body_2->orientation, this->r_2);
    p_1 = this->body_1->position + r_1_wc;
    p_2 = this->body_2->position + r_2_wc;
    delta_p = p_2 - p_1;
    this->current_position = ti::dot(delta_p, moving_axis_wc);
    if (this->type != FREE)
    {
        
        if (this->type == DRIVEN_BY_SPEED){
            this->target_position = this->current_position + this->target_speed* time_step;
        }

        vec3 delta_x = (this->target_position - this->current_position) * moving_axis_wc;

        this->drive_joint_constraint->set_value(delta_x);

        this->drive_joint_constraint->apply_constraint(inverse_time_step);

    }
    
}

void PrismaticJointConstraint::apply_joint_damping(scalar time_step){
    
    vec3 delta_v = (this->body_2->linear_velocity - this->body_1->linear_velocity) * std::min(this->damping * time_step, 1.0);

    if (ti::magnitude(delta_v) < EPSILON) {return ;}

    vec3 r_1_wc = ti::rotate(this->body_1->orientation, this->r_1);
    vec3 r_2_wc = ti::rotate(this->body_2->orientation, this->r_2);

    vec3 n = ti::normalize(delta_v);

    scalar w_1 = this->body_1->get_positional_generalized_inverse_mass(r_1_wc, n);
    scalar w_2 = this->body_2->get_positional_generalized_inverse_mass(r_2_wc, n);

    vec3 impulse = delta_v / (w_1 + w_2);

    this->body_1->apply_positional_velocity_constraint_impulse(impulse, r_1_wc);
    this->body_2->apply_positional_velocity_constraint_impulse(-impulse, r_2_wc);
    
}

void PrismaticJointConstraint::set_traget_position(scalar position){
    this->target_position = position;
}

void PrismaticJointConstraint::set_target_speed(scalar speed){
    this->target_speed = speed;
}

void PrismaticJointConstraint::reset_lagrange_multipliers(){
    this->attachment_point_constraint->reset_lagrange_multiplier();
    this->drive_joint_constraint->reset_lagrange_multiplier();
    this->aligned_constraint->reset_lagrange_multiplier();
}

PrismaticJointInfo PrismaticJointConstraint::get_info(void){
    PrismaticJointInfo info= PrismaticJointInfo{
        .position = this->body_1->position + ti::rotate(this->body_1->orientation, this->r_1),
        .moving_axis = ti::rotate(this->body_1->orientation, this->moving_axis),
        .force = this->force,
        .torque = this->torque,
        .current_position = this->current_position,
        .limited = this->limited,
        .upper_limit = this->upper_limit,
        .lower_limit = this->lower_limit};

    return info;
}

scalar PrismaticJointConstraint::get_current_position(void){
    return this->current_position;
}