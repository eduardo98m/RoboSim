#include "ContactConstraint.hpp"



ContactConstraint::ContactConstraint(Body *body_1,
                                     Body *body_2)
{
    this->body_1 = body_1;
    this->body_2 = body_2;

    vec3 r_1(0.0, 0.0, 0.0);
    vec3 r_2(0.0, 0.0, 0.0);

    this->tangencial_constraint = new PositionalConstraint(body_1, body_2, r_1, r_2, 0, 0);
    this->normal_constraint = new PositionalConstraint(body_1, body_2, r_1, r_2, 0, 0);
}

void ContactConstraint::apply_constraint(const CollisionResponseData &collision_response, scalar inverse_time_step){
    if (!collision_response.collision){return;}

    vec3 r_1_wc = collision_response.contact_point_1 - this->body_1->position;
	vec3 r_2_wc = collision_response.contact_point_2 - this->body_2->position;

    vec3 r_1 = ti::rotate(ti::inverse(this->body_1->orientation), r_1_wc);
    vec3 r_2 = ti::rotate(ti::inverse(this->body_1->orientation), r_2_wc);
    
    this->tangencial_constraint->set_constraint_positions(r_1, r_2);
    this->normal_constraint->set_constraint_positions(r_1, r_2);

    vec3 p_1 = this->body_1->position + ti::rotate(this->body_1->orientation, r_1);
    vec3 p_2 = this->body_2->position + ti::rotate(this->body_2->orientation, r_2);

    vec3 p_1_prev = this->body_1->prev_position + ti::rotate(this->body_1->prev_orientation, r_1);
    vec3 p_2_prev = this->body_2->prev_position + ti::rotate(this->body_2->prev_orientation, r_2);

    vec3 n = collision_response.normal;

    vec3 delta_p = (p_1 - p_1_prev) - (p_2 - p_2_prev);
    vec3 delta_p_tangencial = delta_p - ti::dot(delta_p, n) * n; 

    scalar lambda_n = this->normal_constraint->get_lagrange_multiplier();
    scalar lambda_t = this->tangencial_constraint->get_lagrange_multiplier();

    this->tangencial_constraint->set_value(n * collision_response.penetration_depth);
    this->tangencial_constraint->apply_constraint(inverse_time_step);
    
    if (lambda_t < lambda_n * this->static_fricction_coeff){
        this->tangencial_constraint->set_value(delta_p_tangencial);
        this->tangencial_constraint->apply_constraint(inverse_time_step);
    }
}