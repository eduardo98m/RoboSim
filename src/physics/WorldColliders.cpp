#include "World.hpp"
using namespace robosim;

size_t World::add_collider(Collider collider)
{

    this->colliders.push_back(collider);

    return this->colliders.size() - 1;
}

size_t World::attach_box_collider(size_t id, vec3 half_extents, vec3 position, quat orientation, scalar restitution, scalar dynamic_friction, scalar static_friction,
                                  bool update_inertia)
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
        .body_id = id};

    size_t col_id = this->add_collider(collider);

    if (update_inertia)
    {
        this->recalculate_inertia_with_colliders(id);
    }

    return col_id;
}

size_t World::attach_sphere_collider(size_t id, scalar radius, vec3 position, quat orientation, scalar restitution, scalar dynamic_friction, scalar static_friction, bool update_inertia)
{

    Collider collider = Collider{
        .geom = std::make_shared<hpp::fcl::Sphere>(radius),
        .pos = position,
        .rot = orientation,
        .restitution = restitution,
        .static_fricition = static_friction,
        .dynamic_friction = dynamic_friction,
        .body_id = id};

    size_t col_id = this->add_collider(collider);

    if (update_inertia)
    {
        this->recalculate_inertia_with_colliders(id);
    }

    return col_id;
}

size_t World::attach_capsule_collider(size_t id, scalar radius, scalar height, vec3 position, quat orientation, scalar restitution, scalar dynamic_friction, scalar static_friction, bool update_inertia)
{

    Collider collider = Collider{
        .geom = std::make_shared<hpp::fcl::Capsule>(radius, height),
        .pos = position,
        .rot = orientation,
        .restitution = restitution,
        .static_fricition = static_friction,
        .dynamic_friction = dynamic_friction,
        .body_id = id};

    size_t col_id = this->add_collider(collider);

    if (update_inertia)
    {
        this->recalculate_inertia_with_colliders(id);
    }

    return col_id;
}

size_t World::attach_cylinder_collider(size_t id, scalar radius, scalar height, vec3 position, quat orientation, scalar restitution, scalar dynamic_friction, scalar static_friction, bool update_inertia)
{

    Collider collider = Collider{
        .geom = std::make_shared<hpp::fcl::Cylinder>(radius, height),
        .pos = position, //+ ti::rotate(orientation, {0.0, 0.0, -height*0.5}),
        .rot = orientation,//* ti::quat_from_axis_angle({0.0, 0.0, 1.0}, PI/2) * ti::quat_from_axis_angle({0.0, 1.0, 0.0}, PI/2),
        .restitution = restitution,
        .static_fricition = static_friction,
        .dynamic_friction = dynamic_friction,
        .body_id = id};

    size_t col_id = this->add_collider(collider);

    if (update_inertia)
    {
        this->recalculate_inertia_with_colliders(id);
    }

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
        .body_id = id};

    size_t col_id = this->add_collider(collider);

    return col_id;
}

size_t World::attach_heightmap_collider(size_t id,
                                        scalar x_scale,
                                        scalar y_scale,
                                        std::vector<scalar> heightdata,
                                        size_t x_dims,
                                        size_t y_dims,
                                        scalar restitution,
                                        scalar dynamic_friction,
                                        scalar static_friction)
{

    // Create an Eigen matrix to hold the height data
    hpp::fcl::MatrixXf heightdata_mat(x_dims, y_dims);

    scalar min_value = std::numeric_limits<scalar>::max();

    // Populate the matrix with the provided height data
    for (size_t i = 0; i < x_dims; ++i)
    {
        for (size_t j = 0; j < y_dims; ++j)
        {
            // Assuming the heightdata is stored in row-major order
            heightdata_mat(i, j) = heightdata[i * y_dims + j];
            min_value = ti::min(min_value, heightdata[i * y_dims + j]);
        }
    }

    Collider collider = Collider{
        .geom = std::make_shared<hpp::fcl::HeightField<hpp::fcl::AABB>>(x_scale, y_scale, heightdata_mat, min_value),
        // .pos = position,
        // .rot = orientation,
        .restitution = restitution,
        .static_fricition = static_friction,
        .dynamic_friction = dynamic_friction,
        .body_id = id};

    size_t col_id = this->add_collider(collider);

    return col_id;
}

void World::recalculate_inertia_with_colliders(size_t body_id)
{

    int n_col = this->get_number_of_colliders();

    vec3 body_pos = this->get_body_position(body_id);
    quat body_ori = this->get_body_orientation(body_id);

    scalar volume = 0.0;
    mat3 intertia = mat3{0.0, 0.0, 0.0,
                         0.0, 0.0, 0.0,
                         0.0, 0.0, 0.0};

    for (size_t col_id = 0; col_id < n_col; col_id++)
    {

        if (this->colliders[col_id].body_id != body_id)
        {
            continue;
        };

        Collider *col = &this->colliders[col_id];

        hpp::fcl::CollisionGeometry *geom = col->geom.get();

        mat3 rot_mat = ti::mat3_cast(col->rot * body_ori);
        vec3 pos = col->pos + body_pos;

        mat3 D = {
            pos.y * pos.y + pos.z * pos.z, -pos.x * pos.y, -pos.x * pos.z,
            -pos.x * pos.y, pos.x * pos.x + pos.z * pos.z, -pos.y * pos.z,
            -pos.x * pos.z, -pos.y * pos.z, pos.x * pos.x + pos.y * pos.y};

        mat3 I_com = ti::mat3_from_eigen(geom->computeMomentofInertiaRelatedToCOM());

        scalar V = geom->computeVolume();

        intertia += rot_mat * I_com * ti::transpose(rot_mat) + V * D;

        volume += V;
    }

    scalar density = this->bodies[body_id].get_mass() / volume;
    this->bodies[body_id].set_intertia_tensor(intertia * density);
}

// TODO : Fix this one
// void World::set_heightmap_collider(size_t id, scalar x_scale, scalar y_scale, std::vector<scalar> heightdata, size_t x_dims, size_t y_dims)
// {
//     this->bodies[id].set_heightmap_collider(x_scale, y_scale, heightdata, x_dims, y_dims);
// }

std::pair<vec3, quat> World::get_collider_pose(size_t id)
{

    Collider *col = &this->colliders[id];
    Body *body = &this->bodies[col->body_id];

    quat ori = body->orientation * col->rot;
    vec3 pos = body->position + ti::rotate(body->orientation, col->pos);
    

    return std::make_pair(pos, ori);
}
