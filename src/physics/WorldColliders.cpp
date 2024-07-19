#include "World.hpp"
using namespace robosim;

size_t World::add_collider(Collider collider)
{

    this->colliders.push_back(collider);

    return this->colliders.size() - 1;
}

size_t World::attach_box_collider(size_t id, vec3 half_extents, vec3 position, quat orientation, scalar restitution, scalar dynamic_friction, scalar static_friction)
{

    Collider collider = Collider{
        .geom = std::make_shared<hpp::fcl::Box>(
            half_extents.x * 2.0,
            half_extents.y * 2.0,
            half_extents.z * 2.0),
        .pos = position,
        .rot = orientation,
        .restitution = restitution,
        .static_fricition = static_friction,
        .dynamic_friction = dynamic_friction,
        .body_id = id
        };

    size_t col_id = this->add_collider(collider);

    return col_id;
}

size_t World::attach_sphere_collider(size_t id, scalar radius, vec3 position, quat orientation, scalar restitution, scalar dynamic_friction, scalar static_friction)
{

    

    Collider collider = Collider{
        .geom = std::make_shared<hpp::fcl::Sphere>(radius),
        .pos = position,
        .rot = orientation,
        .restitution = restitution,
        .static_fricition = static_friction,
        .dynamic_friction = dynamic_friction,
        .body_id = id
        };

    size_t col_id = this->add_collider(collider);

    return col_id;
}

size_t World::attach_capsule_collider(size_t id, scalar radius, scalar height, vec3 position, quat orientation, scalar restitution, scalar dynamic_friction, scalar static_friction)
{

    Collider collider = Collider{
        .geom = std::make_shared<hpp::fcl::Capsule>(radius, height),
        .pos = position,
        .rot = orientation,
        .restitution = restitution,
        .static_fricition = static_friction,
        .dynamic_friction = dynamic_friction,
        .body_id = id
        };

    size_t col_id = this->add_collider(collider);

    return col_id;
}

size_t World::attach_cylinder_collider(size_t id, scalar radius, scalar height, vec3 position, quat orientation, scalar restitution, scalar dynamic_friction, scalar static_friction)
{

    Collider collider = Collider{
        .geom = std::make_shared<hpp::fcl::Cylinder>(radius, height),
        .pos = position,
        .rot = orientation,
        .restitution = restitution,
        .static_fricition = static_friction,
        .dynamic_friction = dynamic_friction,
        .body_id = id
        };

    size_t col_id = this->add_collider(collider);

    return col_id;
}

size_t World::attach_plane_collider(size_t id, vec3 normal, scalar offset, scalar restitution, scalar dynamic_friction, scalar static_friction)
{
    Collider collider = Collider{
        .geom = std::make_shared<hpp::fcl::Halfspace>(normal.x, normal.y, normal.z, offset),
        // .pos = position,
        // .rot = orientation,
        .restitution = restitution,
        .static_fricition = static_friction,
        .dynamic_friction = dynamic_friction,
        .body_id = id
    };

    size_t col_id = this->add_collider(collider);

    return col_id;
}

// TODO : Fix this one
// void World::set_heightmap_collider(size_t id, scalar x_scale, scalar y_scale, std::vector<scalar> heightdata, size_t x_dims, size_t y_dims)
// {
//     this->bodies[id].set_heightmap_collider(x_scale, y_scale, heightdata, x_dims, y_dims);
// }


std::pair<vec3, quat> World::get_collider_pose(size_t id){
    
    Collider* col = &this->colliders[id];
    Body* body = &this->bodies[col->body_id];

    vec3 pos = body->position + ti::rotate(body->orientation, col->pos);
    quat ori  = body->orientation * col->rot;


    return std::make_pair(pos, ori);
}
