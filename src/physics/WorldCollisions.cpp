#include "World.hpp"
using namespace robosim;

// Narrow phase collisions

void World::broad_phase_collision_detection(void)
{
    //this->potential_collision_pairs.clear();

    this->aabb_tree.collisions.clear();
    this->aabb_tree.clear();
    
    size_t n_col = this->colliders.size();

    std::vector<size_t> plane_colliders(0);

    for (size_t i = 0; i < n_col; i++)
    {   
        Collider *col = &this->colliders[i];

        if (std::dynamic_pointer_cast<hpp::fcl::Plane>(col->geom)){
            plane_colliders.push_back(i);
            continue;
        }
        
        std::shared_ptr<Body> body = this->bodies[col->body_id];

        AABB aabb = compute_AABB(col->geom,
                         body->position + ti::rotate(body->orientation, col->pos),
                         body->orientation * col->rot);
        vec3 expansion_factor = ti::abs(2.0 * body->linear_velocity * this->timestep);
        aabb = AABB{
            .min = aabb.min - expansion_factor,
            .max = aabb.max + expansion_factor,
        };

        this->aabb_tree.insert(aabb,i);
    }

    for (size_t id : plane_colliders){
        Collider *col = &this->colliders[id];
        auto plane = std::dynamic_pointer_cast<hpp::fcl::Plane>(col->geom);
        this->aabb_tree.query_plane(*plane, id);
    }
    
    // This is an O(n^2) approach 
    // for (size_t i = 0; i < n_col-1; i++)
    // {
    //     for (size_t j = (i + 1); j < n_col; j++)
    //     {

    //         Collider *col_1 = &this->colliders[i];
    //         Collider *col_2 = &this->colliders[j];

    //         if (this->can_collide(col_1->body_id, col_2->body_id) && col_1->body_id != col_2->body_id)
    //         {
    //             bool aabb_col = this->check_aabb_collision(this->colliders[i],
    //                                                        this->bodies[col_1->body_id],
    //                                                        this->colliders[j],
    //                                                        this->bodies[col_2->body_id]);

    //             if (aabb_col)
    //             {
    //                 potential_collision_pairs.push_back({i, j});
    //             }
    //         }
    //     }
    // }
}

bool World::check_aabb_collision(const Collider &col_1, const Body &body_1, const Collider &col_2, const Body &body_2)
{

    // Check this for heightfields
    if (auto plane = std::dynamic_pointer_cast<hpp::fcl::Plane>(col_1.geom))
    {
        AABB body_2_aabb = compute_AABB(col_2.geom,
                                        body_2.position + ti::rotate(body_2.orientation, col_2.pos),
                                        body_2.orientation * col_2.rot);

        vec3 expansion_factor = ti::abs(2.0 * body_2.linear_velocity * this->timestep);
        body_2_aabb = AABB{
            .min = body_2_aabb.min - expansion_factor,
            .max = body_2_aabb.max + expansion_factor,
        };
        return check_broad_phase_collision(*plane, body_2_aabb);
    }
    else if (auto plane = std::dynamic_pointer_cast<hpp::fcl::Plane>(col_2.geom))
    {
        AABB body_1_aabb = compute_AABB(col_1.geom,
                                        body_1.position + ti::rotate(body_1.orientation, col_1.pos),
                                        body_1.orientation * col_1.rot);

        vec3 expansion_factor = ti::abs(2.0 * body_1.linear_velocity * this->timestep);
        body_1_aabb = AABB{
            .min = body_1_aabb.min - expansion_factor,
            .max = body_1_aabb.max + expansion_factor,
        };
        return check_broad_phase_collision(*plane, body_1_aabb);
    }
    AABB body_1_aabb = compute_AABB(col_1.geom,
                                    body_1.position + ti::rotate(body_1.orientation, col_1.pos),
                                    body_1.orientation * col_1.rot);

    AABB body_2_aabb = compute_AABB(col_2.geom,
                                    body_2.position + ti::rotate(body_2.orientation, col_2.pos),
                                    body_2.orientation * col_2.rot);

    vec3 ef_1 = ti::abs(2.0 * body_1.linear_velocity * this->timestep);
    body_1_aabb = AABB{
        .min = body_1_aabb.min - ef_1,
        .max = body_1_aabb.max + ef_1,
    };

    vec3 ef_2 = ti::abs(2.0 * body_2.linear_velocity * this->timestep);
    body_2_aabb = AABB{
        .min = body_2_aabb.min - ef_2,
        .max = body_2_aabb.max + ef_2,
    };

    return check_broad_phase_collision(body_1_aabb, body_2_aabb);
}

void World::narrow_phase_collision_detection_and_response(scalar inverse_time_step)
{

    this->contact_constraints.clear();

    for (std::pair<size_t, size_t> col_pair : this->aabb_tree.collisions)
    {
        Collider *col_1 = &this->colliders[col_pair.first];
        Collider *col_2 = &this->colliders[col_pair.second];

        if (!this->can_collide(col_1->body_id, col_2->body_id) ||  col_1->body_id == col_2->body_id){
            continue;
        }
        std::shared_ptr<Body> body_1 = this->bodies[col_1->body_id];
        std::shared_ptr<Body> body_2 = this->bodies[col_2->body_id];

        hpp::fcl::CollisionResult col_res;
        hpp::fcl::CollisionRequest col_req = hpp::fcl::CollisionRequest(hpp::fcl::CollisionRequestFlag::CONTACT, 16);

        hpp::fcl::collide(col_1->geom.get(),
                          ti::get_eigen_transform(body_1->position + ti::rotate(body_1->orientation, col_1->pos),
                                                  body_1->orientation * col_1->rot),
                          col_2->geom.get(),
                          ti::get_eigen_transform(body_2->position + ti::rotate(body_2->orientation, col_2->pos),
                                                  body_2->orientation * col_2->rot),
                          col_req,
                          col_res);

        if (col_res.isCollision())
        {
            int n_contacts = col_res.numContacts();

            for (int i = 0; i < n_contacts; i++)
            {

                hpp::fcl::Contact contact = col_res.getContact(i);
                scalar penetration_depth = contact.penetration_depth;
                vec3 normal = ti::from_eigen(contact.normal);
                vec3 contact_point_1 = ti::from_eigen(contact.pos) + normal * penetration_depth * 0.5;
                vec3 contact_point_2 = ti::from_eigen(contact.pos) - normal * penetration_depth * 0.5;

                // Create a contact and solve it

                ContactConstraint contact_constraint = ContactConstraint(body_1,
                                                                         body_2,
                                                                         normal,
                                                                         contact_point_1,
                                                                         contact_point_2,
                                                                         (col_1->static_fricition + col_2->static_fricition) * 0.5,
                                                                         (col_1->dynamic_friction + col_2->dynamic_friction) * 0.5,
                                                                         ti::min(col_1->restitution, col_2->restitution));

                contact_constraint.apply_constraint_position_level(inverse_time_step);
                // Solve the contact (Position solve)
                this->contact_constraints.push_back(contact_constraint);
            }
        }
    }
}