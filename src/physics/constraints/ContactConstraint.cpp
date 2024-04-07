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

    this->static_fricction_coeff = 0.5 * (body_1->static_fricction_coeff + body_2->static_fricction_coeff);
    this->dynamic_fricction_coeff = 0.5 * (body_1->dynamic_fricction_coeff + body_2->dynamic_fricction_coeff);
    this->restitution = ti::min(body_1->restitution, body_2->restitution);
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
    vec3 p_1 = this->collision_response.contact_point_1;
    vec3 p_2 = this->collision_response.contact_point_2;
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

    this->normal_constraint->set_constraint_positions(r_1, r_2);
    this->normal_constraint->set_value(n * d);
    this->normal_constraint->apply_constraint(inverse_time_step);

    // We calculate the relative velocity:
    vec3 v_n = (this->body_1->linear_velocity + ti::cross(this->body_1->angular_velocity, r_1_wc)) -
               (this->body_2->linear_velocity + ti::cross(this->body_2->angular_velocity, r_2_wc));

    this->relative_velocity = ti::dot(v_n, n);

    
    // Note this are recalculated for the previous positions
    r_1_wc = p_1 - this->body_1->prev_position;
    r_2_wc = p_2 - this->body_2->prev_position;

    r_1 = ti::rotate(ti::inverse(this->body_1->prev_orientation), r_1_wc);
    r_2 = ti::rotate(ti::inverse(this->body_2->prev_orientation), r_2_wc);

    vec3 prev_p_1 = this->body_1->prev_position + ti::rotate(this->body_1->prev_orientation, r_1);
    vec3 prev_p_2 = this->body_2->prev_position + ti::rotate(this->body_2->prev_orientation, r_2);

    vec3 delta_p = (p_1 - prev_p_1) - (p_2 - prev_p_2);

    vec3 delta_p_tangencial = delta_p - (ti::dot(delta_p, n)) * n;

    scalar lambda_t = this->tangencial_constraint->compute_lagrange_multiplier(inverse_time_step);
    scalar lambda_n = this->normal_constraint->get_lagrange_multiplier();

    if (lambda_t > lambda_n * this->static_fricction_coeff)
    {
        this->tangencial_constraint->set_constraint_positions(r_1, r_2);
        this->tangencial_constraint->set_value(delta_p_tangencial);
        this->tangencial_constraint->apply_constraint(inverse_time_step);
    }

    this->normal_force = this->normal_constraint->force;
}

void ContactConstraint::apply_constraint_velocity_level(scalar time_step)
{
    if (!this->broad_phase_detection)
    {
        this->collision = false;
        return;
    }

    vec3 n = this->collision_response.normal;
    vec3 p_1 = this->collision_response.contact_point_1;
    vec3 p_2 = this->collision_response.contact_point_2;
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
        scalar friction = ti::min(-this->dynamic_fricction_coeff * this->normal_constraint->get_lagrange_multiplier() / time_step,
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

void ContactConstraint::reset_lagrange_multipliers(void)
{
    this->tangencial_constraint->reset_lagrange_multiplier();
    this->normal_constraint->reset_lagrange_multiplier();
}

AABB ContactConstraint::get_aabb(Body *body)
{
    return compute_AABB(body->collider_info, body->position, body->orientation);
}

void ContactConstraint::check_broad_phase(scalar timestep)
{

    if (auto plane = std::dynamic_pointer_cast<hpp::fcl::Plane>(this->body_1->collider_info))
    {
        AABB body_2_aabb = this->get_aabb(this->body_2);

        vec3 expand_factor = ti::abs(2.0 * body_2->linear_velocity * timestep);
        body_2_aabb = AABB{
            .min = body_2_aabb.min - expand_factor,
            .max = body_2_aabb.max + expand_factor,
        };
        this->broad_phase_detection = check_broad_phase_collision(*plane, body_2_aabb);
        return;
    }
    else if (auto plane = std::dynamic_pointer_cast<hpp::fcl::Plane>(this->body_2->collider_info))
    {
        AABB body_1_aabb = this->get_aabb(this->body_1);
        vec3 expand_factor = ti::abs(2.0 * body_1->linear_velocity * timestep);
        body_1_aabb = AABB{
            .min = body_1_aabb.min - expand_factor,
            .max = body_1_aabb.max + expand_factor,
        };
        this->broad_phase_detection = check_broad_phase_collision(*plane, body_1_aabb);
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

    hpp::fcl::CollisionResult col_res;
    hpp::fcl::CollisionRequest col_req = hpp::fcl::CollisionRequest(hpp::fcl::CollisionRequestFlag::CONTACT, 16);

    hpp::fcl::collide(this->body_1->collider_info.get(),
                      ti::get_eigen_transform(this->body_1->position,
                                              this->body_1->orientation),
                      this->body_2->collider_info.get(),
                      ti::get_eigen_transform(this->body_2->position,
                                              this->body_2->orientation),
                      col_req,
                      col_res);

    if (col_res.isCollision())
    {
        vec3 normal = {0.0, 0.0, 0.0};
        vec3 contact_point_1 = {0.0, 0.0, 0.0};
        vec3 contact_point_2 = {0.0, 0.0, 0.0};

        int n_contacts = col_res.numContacts();

        if (n_contacts == 1)
        {
            hpp::fcl::Contact contact = col_res.getContact(0);
            scalar penetration_depth = contact.penetration_depth;
            normal = ti::from_eigen(contact.normal);
            contact_point_1 = ti::from_eigen(contact.pos) + normal * penetration_depth * 0.5;
            contact_point_2 = ti::from_eigen(contact.pos) - normal * penetration_depth * 0.5;
        }
        else
        {
            scalar max_penetration_depth = 0.0;
            hpp::fcl::Contact max_contact;

            for (int i = 0; i < n_contacts; i++)
            {
                hpp::fcl::Contact contact = col_res.getContact(i);
                scalar penetration_depth = contact.penetration_depth;

                if (penetration_depth > max_penetration_depth)
                {
                    max_penetration_depth = penetration_depth;
                    max_contact = contact;
                }
            }

            normal = ti::from_eigen(max_contact.normal);
            contact_point_1 = ti::from_eigen(max_contact.pos) + normal * max_penetration_depth* 0.5;
            contact_point_2 = ti::from_eigen(max_contact.pos) - normal * max_penetration_depth* 0.5;
        }

        this->collision_response = ContactPoint{
            .normal = ti::normalize(normal),
            .contact_point_1 = contact_point_1,
            .contact_point_2 = contact_point_2};
    }
    else
    {
        this->collision_response = ContactPoint();
    }

    col_res.clear();
}