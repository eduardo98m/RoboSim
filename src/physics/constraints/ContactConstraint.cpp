#include "ContactConstraint.hpp"

ContactConstraint::ContactConstraint(Body *body_1,
                                     Body *body_2)
{
    this->body_1 = body_1;
    this->body_2 = body_2;

    vec3 r_1(0.0, 0.0, 0.0);
    vec3 r_2(0.0, 0.0, 0.0);

    this->tangencial_constraint = new PositionalConstraint(body_1, body_2, r_1, r_2, 0.0, 0.0);
    this->normal_constraint = new PositionalConstraint(body_1, body_2, r_1, r_2, 0.0, 0.0);

    this->static_fricction_coeff = 12.5;
}

void ContactConstraint::apply_constraint(scalar inverse_time_step)
{
    if (!this->broad_phase_detection)
    {
        this->collision = false;
        return;
    }

    this->calculate_narrow_phase_collision_response();

    vec3 n = this->collision_response.normal;
    scalar d = ti::dot((this->collision_response.contact_point_1 - this->collision_response.contact_point_2), n);

    if (d <= 0)
    {
        this->collision = false;
        return;
    }
    this->collision = true;

    vec3 r_1_wc = this->collision_response.contact_point_1 - this->body_1->position;
    vec3 r_2_wc = this->collision_response.contact_point_2 - this->body_2->position;

    vec3 r_1 = ti::rotate(ti::inverse(this->body_1->orientation), r_1_wc);
    vec3 r_2 = ti::rotate(ti::inverse(this->body_2->orientation), r_2_wc);

    vec3 p_1 = this->body_1->position + ti::rotate(this->body_1->orientation, r_1);
    vec3 p_2 = this->body_2->position + ti::rotate(this->body_2->orientation, r_2);

    this->normal_constraint->set_constraint_positions(r_1, r_2);
    this->normal_constraint->set_value(n * d);
    this->normal_constraint->apply_constraint(inverse_time_step);

    // We calculate the relative velocity:
    vec3 v_n = (this->body_1->linear_velocity + ti::cross(this->body_1->angular_velocity, r_1)) -
               (this->body_2->linear_velocity + ti::cross(this->body_2->angular_velocity, r_2));

    this->relative_velocity = ti::dot(v_n, n);

    scalar lambda_t = this->normal_constraint->get_lagrange_multiplier();
    scalar lambda_n = this->normal_constraint->get_lagrange_multiplier();

    // Note this are recalculated for the previous positions
    r_1_wc = this->collision_response.contact_point_1 - this->body_1->prev_position;
    r_2_wc = this->collision_response.contact_point_2 - this->body_2->prev_position;

    r_1 = ti::rotate(ti::inverse(this->body_1->prev_orientation), r_1_wc);
    r_2 = ti::rotate(ti::inverse(this->body_2->prev_orientation), r_2_wc);

    vec3 prev_p_1 = this->body_1->prev_position + ti::rotate(this->body_1->prev_orientation, r_1);
    vec3 prev_p_2 = this->body_2->prev_position + ti::rotate(this->body_2->prev_orientation, r_2);

    vec3 delta_p = (p_1 - prev_p_1) - (p_2 - prev_p_2);

    vec3 delta_p_tangencial = delta_p - (ti::dot(delta_p, n)) * n;

    if (lambda_t < lambda_n * this->static_fricction_coeff)
    {
        this->tangencial_constraint->set_value(delta_p_tangencial);
        this->tangencial_constraint->apply_constraint(inverse_time_step);
    }

    this->normal_force = this->normal_constraint->force;
}

void ContactConstraint::apply_constraint_velocity_level(scalar time_step)
{
    if (!this->broad_phase_detection)
    {
        return;
    }
    vec3 n = this->collision_response.normal;
    scalar d = ti::dot((this->collision_response.contact_point_1 - this->collision_response.contact_point_2), n);
    if (d <= 0)
    {
        return;
    }

    vec3 r_1_wc = this->collision_response.contact_point_1 - this->body_1->position;
    vec3 r_2_wc = this->collision_response.contact_point_2 - this->body_2->position;

    vec3 r_1 = ti::rotate(ti::inverse(this->body_1->orientation), r_1_wc);
    vec3 r_2 = ti::rotate(ti::inverse(this->body_2->orientation), r_2_wc);
    // Recalculate the new velocity
    vec3 v = this->body_1->linear_velocity + ti::cross(this->body_1->angular_velocity, r_1) -
             this->body_2->linear_velocity + ti::cross(this->body_2->angular_velocity, r_2);

    scalar v_n = ti::dot(v, n);
    vec3 v_t = v - n * v_n;
    vec3 tangencial_direction = (ti::magnitude(v_t) > EPSILON) ? ti::normalize(v_t) : (v_t * 0.0);
    vec3 tangencial_correction = -tangencial_direction * ti::min(time_step * this->dynamic_fricction_coeff * ti::magnitude(this->normal_force),
                                                              ti::magnitude(v_t));

                                                
    if (ti::magnitude(tangencial_correction) > EPSILON)
    {   
        scalar w_1 = this->body_1->get_positional_generalized_inverse_mass(r_1_wc, tangencial_direction);
        scalar w_2 = this->body_2->get_positional_generalized_inverse_mass(r_2_wc, tangencial_direction);
        vec3 tangencial_impulse = tangencial_correction/(w_1 + w_2);
        this->body_1->apply_positional_velocity_constraint_impulse(tangencial_impulse, r_1_wc);
        this->body_2->apply_positional_velocity_constraint_impulse(-tangencial_impulse, r_2_wc);
    };
    // Restitution

    scalar restitution = 0.75;
    // if (v_n < EPSILON){restitution = 0.0;}
    // TODO : Change this so we use the simulator gravity
    if (ti::abs(v_n) <= 2.0 * 9.8 * time_step)
    {
        restitution = 0.0;
    }
    vec3 delta_v = n * (-v_n + ti::min(-restitution * this->relative_velocity, 0.0));
    // Calculte the generalized inverse mass of the bodies
    if (ti::magnitude(delta_v) > EPSILON)
    {   
        scalar w_1 = this->body_1->get_positional_generalized_inverse_mass(r_1_wc, n);
        scalar w_2 = this->body_2->get_positional_generalized_inverse_mass(r_2_wc, n);
        vec3 restitution_impulse = delta_v / (w_1 + w_2);
        this->body_1->apply_positional_velocity_constraint_impulse(restitution_impulse, r_1_wc);
        this->body_2->apply_positional_velocity_constraint_impulse(-restitution_impulse, r_2_wc);
    }
}

void ContactConstraint::reset_lagrange_multipliers(void)
{
    this->tangencial_constraint->reset_lagrange_multiplier();
    this->normal_constraint->reset_lagrange_multiplier();
}

AABB ContactConstraint::get_aabb(Body *body)
{
    ShapeInfo info = body->collider_info;

    if (info.type == ShapeType::CAPSULE)
    {
        return compute_AABB(*info.capsule, body->position, body->orientation);
    }

    if (info.type == ShapeType::SPHERE)
    {
        return compute_AABB(*info.sphere, body->position, body->orientation);
    }

    if (info.type == ShapeType::BOX)
    {
        return compute_AABB(*info.box, body->position, body->orientation);
    }

    return AABB{.min = vec3{0.0, 0.0, 0.0}, .max = vec3{0.0, 0.0, 0.0}};
}

void ContactConstraint::check_broad_phase(scalar timestep)
{

    if (this->body_1->collider_info.type == ShapeType::PLANE)
    {
        AABB body_2_aabb = this->get_aabb(this->body_2);

        vec3 expand_factor = ti::abs(2.0 * body_2->linear_velocity * timestep);
        body_2_aabb = AABB{
            .min = body_2_aabb.min - expand_factor,
            .max = body_2_aabb.max + expand_factor,
        };
        this->broad_phase_detection = check_broad_phase_collision(*body_1->collider_info.plane, body_2_aabb);
        return;
    }
    else if (this->body_2->collider_info.type == ShapeType::PLANE)
    {
        AABB body_1_aabb = this->get_aabb(this->body_1);
        vec3 expand_factor = ti::abs(2.0 * body_1->linear_velocity * timestep);
        body_1_aabb = AABB{
            .min = body_1_aabb.min - expand_factor,
            .max = body_1_aabb.max + expand_factor,
        };
        this->broad_phase_detection = check_broad_phase_collision(body_1_aabb, *body_2->collider_info.plane);
        return;
    }
    AABB body_1_aabb = this->get_aabb(this->body_1);
    AABB body_2_aabb = this->get_aabb(this->body_2);

    vec3 ef_1 = ti::abs(2.0 * body_1->linear_velocity * timestep);
    body_1_aabb = AABB{
        .min = body_1_aabb.min - ef_1,
        .max = body_1_aabb.max + ef_1,
    };

    vec3 ef_2 = ti::abs(2.0 * body_2->linear_velocity * timestep);
    body_2_aabb = AABB{
        .min = body_2_aabb.min - ef_2,
        .max = body_2_aabb.max + ef_2,
    };

    this->broad_phase_detection = check_broad_phase_collision(body_1_aabb, body_2_aabb);
};

void ContactConstraint::calculate_narrow_phase_collision_response(void)
{
    if (body_1->collider_info.type == ShapeType::SPHERE &&
        body_2->collider_info.type == ShapeType::SPHERE)
    {
        this->collision_response = compute_collision_response(
            body_1->position,
            body_2->position,
            *body_1->collider_info.sphere,
            *body_2->collider_info.sphere);
    }

    else if (body_1->collider_info.type == ShapeType::PLANE &&
        body_2->collider_info.type == ShapeType::SPHERE)
    {
        this->collision_response = compute_collision_response(
            body_2->position,
            *body_2->collider_info.sphere,
            *body_1->collider_info.plane);

        std::swap(this->collision_response.contact_point_1,
                  this->collision_response.contact_point_2);
    }

    else if (body_1->collider_info.type == ShapeType::SPHERE &&
        body_2->collider_info.type == ShapeType::PLANE)
    {
        this->collision_response = compute_collision_response(
            body_1->position,
            *body_1->collider_info.sphere,
            *body_2->collider_info.plane);
    }

    else if (body_1->collider_info.type == ShapeType::BOX &&
        body_2->collider_info.type == ShapeType::BOX)
    {
        // this->collision_response = compute_collision_response(
        //     body_1->position,
        //     body_1->orientation,
        //     body_2->position,
        //     body_2->orientation,
        //     *body_1->collider_info.box,
        //     *body_2->collider_info.box);
    }

    else if (body_1->collider_info.type == ShapeType::PLANE &&
        body_2->collider_info.type == ShapeType::BOX)
    {
        this->collision_response = compute_collision_response(
            body_2->position,
            body_2->orientation,
            *body_2->collider_info.box,
            *body_1->collider_info.plane);
        
        std::swap(this->collision_response.contact_point_1,
                  this->collision_response.contact_point_2);

        
    }

    else if (body_1->collider_info.type == ShapeType::BOX &&
        body_2->collider_info.type == ShapeType::PLANE)
    {
        this->collision_response = compute_collision_response(
            body_1->position,
            body_1->orientation,
            *body_1->collider_info.box,
            *body_2->collider_info.plane);
        std::cerr << "Hola\n";
        
        
    }

    else if (body_1->collider_info.type == ShapeType::SPHERE &&
        body_2->collider_info.type == ShapeType::BOX)
    {
        this->collision_response = compute_collision_response(
            body_2->position,
            body_2->orientation,
            body_1->position,
            *body_2->collider_info.box,
            *body_1->collider_info.sphere);
        
        std::swap(this->collision_response.contact_point_1,
                  this->collision_response.contact_point_2);
        
    }

    else if (body_1->collider_info.type == ShapeType::BOX &&
        body_2->collider_info.type == ShapeType::SPHERE)
    {        
        this->collision_response = compute_collision_response(
            body_1->position,
            body_1->orientation,
            body_2->position,
            *body_1->collider_info.box,
            *body_2->collider_info.sphere);
    }
}