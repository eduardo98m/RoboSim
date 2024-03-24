#include "World.hpp"

using namespace robosim;

World::World(scalar timestep, int substeps)
{
    this->timestep = timestep;
    this->substeps = substeps;
};

void World::step()
{
    scalar h = this->timestep / this->substeps;
    scalar inv_h = 1 / h;

    this->broad_phase_collision_detection();
    for (ContactConstraint &constraint : this->contact_constraints)
    {
        constraint.reset_lagrange_multipliers();
    }
    for (int i = 0; i < this->substeps; i++)
    {
        this->update_bodies_position_and_orientation(h);
        this->solve_positions(inv_h, h);
        this->update_bodies_velocities(inv_h);
        this->solve_velocities(h);
    }
}

void World::update_bodies_position_and_orientation(scalar h)
{
    for (Body &body : this->bodies)
    {
        body.update_position_and_orientation(h);
    }
}

void World::update_bodies_velocities(scalar inv_h)
{
    for (Body &body : this->bodies)
    {
        body.update_velocities(inv_h);
    }
}

void World::solve_positions(scalar inv_h, scalar h)
{
    for (ContactConstraint &constraint : this->contact_constraints)
    {
        constraint.apply_constraint(inv_h);
    }
    for (PositionalConstraint &constraint : this->positional_constraints)
    {
        constraint.apply_constraint(inv_h);
    }
    for (RotationalConstraint &constraint : this->rotational_constraints)
    {
        constraint.apply_constraint(inv_h);
    }

    for (RevoluteJointConstraint &constraint : this->revolute_joint_constraints)
    {
        constraint.apply_constraint(inv_h, h);
    }
}
void World::solve_velocities(scalar h)
{
    for (ContactConstraint &constraint : this->contact_constraints)
    {
        constraint.apply_constraint_velocity_level(h);
    }

    for (RevoluteJointConstraint &constraint : this->revolute_joint_constraints)
    {
        constraint.apply_joint_damping(h);
    }
}

scalar World::get_time_step()
{
    return this->timestep;
}

void World::set_gravity(vec3 gravity)
{
    for (auto &body : this->bodies)
    {
        body.set_gravity(gravity);
    }
}

int World::add_body(Body body)
{
    this->bodies.push_back(body);

    return (int)(this->bodies.size() - 1);
}

int World::create_body(vec3 position,
                       quat orientation,
                       vec3 linear_velocity,
                       vec3 angular_velocity,
                       scalar mass,
                       mat3 inertia_tensor,
                       BodyType type)
{
    Body body = Body(position,
                     orientation,
                     linear_velocity,
                     angular_velocity,
                     mass,
                     inertia_tensor,
                     type);

    return World::add_body(body);
}

void World::set_body_plane_collider(int id, vec3 normal, scalar offset)
{
    this->bodies[id].set_plane_collider(normal, offset);
}

void World::set_body_box_collider(int id, vec3 half_extents)
{
    this->bodies[id].set_box_collider(half_extents, true);
}

void World::set_body_cylinder_collider(int id, scalar radius, scalar height)
{
    this->bodies[id].set_cylinder_collider(radius, height, true);
}

void World::set_body_sphere_collider(int id, scalar radius)
{
    this->bodies[id].set_sphere_collider(radius, true);
}

void World::set_body_capsule_collider(int id, scalar radius, scalar height)
{
    this->bodies[id].set_capsule_collider(radius, height);
}

int World::create_positional_constraint(int body_1_id, int body_2_id, vec3 r_1, vec3 r_2, scalar compliance, scalar damping)
{

    PositionalConstraint constraint = PositionalConstraint(&this->bodies[body_1_id],
                                                           &this->bodies[body_2_id],
                                                           r_1,
                                                           r_2,
                                                           compliance,
                                                           damping);

    this->positional_constraints.push_back(constraint);
    return (int)(this->positional_constraints.size() - 1);
}

int World::create_rotational_constraint(int body_1_id, int body_2_id, vec3 r_1, vec3 r_2, scalar compliance, scalar damping)
{

    RotationalConstraint constraint = RotationalConstraint(&this->bodies[body_1_id],
                                                           &this->bodies[body_2_id],
                                                           r_1,
                                                           r_2,
                                                           compliance,
                                                           damping);

    this->rotational_constraints.push_back(constraint);
    return (int)(this->rotational_constraints.size() - 1);
}

int World::create_revolute_constraint(int body_1_id,
                                      int body_2_id,
                                      vec3 aligned_axis,
                                      vec3 r_1,
                                      vec3 r_2,
                                      scalar compliance,
                                      scalar damping,
                                      RevoluteJointType type,
                                      bool limited,
                                      scalar lower_limit,
                                      scalar upper_limit,
                                      bool set_limit_axis,
                                      vec3 limit_axis)
{
    if (!set_limit_axis)
    {
        limit_axis = aligned_axis;
        limit_axis.x += 1.0;
        limit_axis = ti::cross(aligned_axis, limit_axis);
        if (ti::magnitude(limit_axis) < EPSILON)
        {
            limit_axis = aligned_axis;
            limit_axis.y += 1.0;
            limit_axis = ti::cross(aligned_axis, limit_axis);
        }
        limit_axis = ti::normalize(limit_axis);
    }
    assert(ti::magnitude(limit_axis) > EPSILON && "Invalid limit axis `limit_axis` must be perpendicular to the `aligned_axis`");

    RevoluteJointConstraint constraint = RevoluteJointConstraint(&this->bodies[body_1_id],
                                                                 &this->bodies[body_2_id],
                                                                 ti::normalize(aligned_axis),
                                                                 limit_axis,
                                                                 r_1,
                                                                 r_2,
                                                                 compliance,
                                                                 damping,
                                                                 type,
                                                                 limited,
                                                                 lower_limit,
                                                                 upper_limit);

    this->revolute_joint_constraints.push_back(constraint);
    return (int)(this->revolute_joint_constraints.size() - 1);
}

int World::create_contact_constraint(int body_1_id,
                                     int body_2_id)
{

    ContactConstraint constraint = ContactConstraint(&this->bodies[body_1_id],
                                                     &this->bodies[body_2_id]);

    this->contact_constraints.push_back(constraint);
    return (int)(this->contact_constraints.size() - 1);
}

int World::add_rotational_constraint(RotationalConstraint constraint)
{
    this->rotational_constraints.push_back(constraint);
    return (int)(this->rotational_constraints.size() - 1);
}

vec3 World::get_body_position(int id)
{
    return this->bodies[id].position;
}

quat World::get_body_orientation(int id)
{
    return this->bodies[id].orientation;
}

vec3 World::get_body_angular_velocity(int id)
{
    return this->bodies[id].angular_velocity;
}

std::shared_ptr<hpp::fcl::CollisionGeometry> World::get_collider_info(int id)
{
    return this->bodies[id].collider_info;
}

AABB World::get_aabb(int id)
{
    std::shared_ptr<hpp::fcl::CollisionGeometry> info = this->bodies[id].collider_info;

    AABB aabb = compute_AABB(info, this->bodies[id].position, this->bodies[id].orientation);

    vec3 expansion_factor = ti::abs(2.0 * this->bodies[id].linear_velocity * this->timestep);
    aabb = AABB{
        .min = aabb.min - expansion_factor,
        .max = aabb.max + expansion_factor,
    };

    return aabb;
}

void World::collisions_detection_preparations(void)
{

    int n_bodies = this->get_number_of_bodies();

    for (int i = 0; i < (n_bodies - 1); i++)
    {
        for (int j = i + 1; j < n_bodies; j++)
        {
            if (this->can_collide(i, j))
            {
                int id = this->create_contact_constraint(i, j);
                this->body_pair_to_contact_constraint_map.insert({{i, j}, id});
            }
        }
    }
}

bool World::can_collide(size_t bodyA, size_t bodyB) const
{
    // Check if either body belongs to group 0 (default collision group)
    if (collision_groups.count(bodyA) == 0 || collision_groups.count(bodyB) == 0)
    {
        return true; // Default collision for unassigned bodies or group 0
    }
    // Otherwise, check for bit overlap in their group bitsets

    // Otherwise, check for bit overlap in their collision group bitsets
    uint32_t groupA = collision_groups.at(bodyA);
    uint32_t groupB = collision_groups.at(bodyB);
    uint32_t commonGroups = groupA & groupB; // Bitwise AND to find common groups

    return commonGroups != 0;
}

void World::set_collision_group(size_t id, u_int32_t collision_group)
{
    if (collision_group < 0 || collision_group >= MAX_COLLISION_GROUPS)
    {
        // Handle error - invalid collision group
        return;
    }
    this->collision_groups[id] = collision_group;
}

// Collisions
void World::broad_phase_collision_detection(void)
{
    for (auto &constraint : this->contact_constraints)
    {
        constraint.check_broad_phase(this->timestep);
    }
}

std::vector<vec3> World::raycast(vec3 start, vec3 end)
{

    // Create a capsule in hpp::fcl
    scalar lenght = ti::magnitude(end - start);
    scalar radius = 0.001; // if we take 1 m as the unit, this should be 1 [mm]

    std::shared_ptr<hpp::fcl::Capsule> ray_collider = std::make_shared<hpp::fcl::Capsule>(radius, lenght);

    vec3 position = 0.5 * (start + end);
    vec3 direction = ti::normalize(end - start);
    vec3 up = {0.0, 0.0, 1.0};
    vec3 cross = ti::cross(up, direction);
    vec3 axis = cross/ti::magnitude(cross);
    scalar angle  = ti::acos(ti::dot(direction, up));
    quat orientation = ti::quat_from_axis_angle(axis, angle) ;
    
    hpp::fcl::CollisionObject* ray = new hpp::fcl::CollisionObject(ray_collider);
    ray->setTransform(ti::get_eigen_transform(position, orientation));

    ray->computeAABB();

    std::vector<vec3> points;
    // Now we need to check the capsule collider with the rest of the bodies of the world
    for (int i = 0; i < this->bodies.size(); i++)
    {

        hpp::fcl::CollisionObject* col_obj = new hpp::fcl::CollisionObject(this->bodies[i].collider_info);
        col_obj->setTransform(
                ti::get_eigen_transform(this->bodies[i].position,
                                                  this->bodies[i].orientation)
        );
        col_obj->computeAABB();

        if (ray->getAABB().contain(col_obj->getAABB())){
            hpp::fcl::CollisionResult col_res;
            hpp::fcl::CollisionRequest col_req;

            col_req.num_max_contacts = 2;

            hpp::fcl::collide(ray,
                            col_obj,
                            col_req,
                            col_res);

            if (col_res.isCollision())
            {
                vec3 point;
                scalar min_dist = INFINITY;
                for (int i = 0; i< col_res.numContacts(); i++ ){
                    vec3 candidate = ti::from_eigen(col_res.getContact(i).pos);
                    // scalar dist = ti::magnitude(start - candidate);
                    // if (dist < min_dist){
                    //     min_dist = dist;
                    //     point = candidate;
                    // }

                    points.push_back(candidate);
                }
            }
        }
        
    }

    return points;
}

// std::vector<vec3> World::raycast(vec3 center, scalar radius)
// {

//     // Create a capsule in hpp::fcl
//     scalar lenght = ti::magnitude(end - start);
//     scalar radius = 0.001; // if we take 1 m as the unit, this should be 1 [mm]

//     std::shared_ptr<hpp::fcl::Capsule> disc = std::make_shared<hpp::fcl::Capsule>(radius, lenght);

//     vec3 position = 0.5*(start + end);

//     vec3 direction = ti::normalize(end - start);
//     vec3 up = {0.0, 1.0, 0.0};

//     // Create a quaternion representing the capsule orientation
//     quat orientation = ti::quat_from_axis_angle(direction, 0.0f);

//     // Rotate the quaternion by 90 degrees around the up vector to align the capsule with the raycast
//     orientation = orientation * ti::quat_from_axis_angle(up, M_PI / 2.0f);

    
//     hpp::fcl::CollisionObject* ray = new hpp::fcl::CollisionObject(ray_collider);
//     ray->setTransform(ti::get_eigen_transform(position, orientation));

//     ray->computeAABB();

//     std::vector<vec3> points;
//     // Now we need to check the capsule collider with the rest of the bodies of the world
//     for (int i = 0; i < this->bodies.size(); i++)
//     {

//         hpp::fcl::CollisionObject* col_obj = new hpp::fcl::CollisionObject(this->bodies[i].collider_info);
//         col_obj->setTransform(
//                 ti::get_eigen_transform(this->bodies[i].position,
//                                                   this->bodies[i].orientation)
//         );
//         col_obj->computeAABB();

//         if (true){
//             hpp::fcl::CollisionResult col_res;
//             hpp::fcl::CollisionRequest col_req;

//             hpp::fcl::collide(ray_collider.get(),
//                             ti::get_eigen_transform(position, orientation),
//                             this->bodies[i].collider_info.get(),
//                             ti::get_eigen_transform(this->bodies[i].position,
//                                                   this->bodies[i].orientation),
//                             col_req,
//                             col_res);

//             if (col_res.isCollision())
//             {
//                 vec3 point;
//                 scalar min_dist = INFINITY;
//                 for (int i = 0; i< col_res.numContacts(); i++ ){
//                     vec3 candidate = ti::from_eigen(col_res.getContact(i).pos);
//                     scalar dist = ti::magnitude(start - candidate);
//                     if (dist < min_dist){
//                         min_dist = dist;
//                         point = candidate;
//                     }
//                 }
//                 points.push_back(point);
//             }
//         }
        
//     }

//     return points;
// }

// Adding plane:
int World::add_plane(vec3 normal, scalar offset)
{
    normal = ti::normalize(normal);
    this->plane_body_idx = this->create_body(vec3(0.0, 0.0, 0.0),
                                             quat(1.0, 0.0, 0.0, 0.0),
                                             vec3(0.0, 0.0, 0.0),
                                             vec3(0.0, 0.0, 0.0),
                                             1.0,
                                             1.0 * mat3(1.0, 0.0, 0.0,
                                                        0.0, 1.0, 0.0,
                                                        0.0, 0.0, 1.0),
                                             BodyType::STATIC);

    this->set_body_plane_collider(this->plane_body_idx, normal, offset);

    return this->plane_body_idx;
}

// Revolute joints
RevoluteJointInfo World::get_revolute_joint_info(int id)
{
    return this->revolute_joint_constraints[id].get_info();
}

void World::set_revolute_joint_target_angle(int id, scalar angle)
{
    this->revolute_joint_constraints[id].set_traget_angle(angle);
}

void World::set_revolute_joint_target_speed(int id, scalar speed)
{
    this->revolute_joint_constraints[id].set_target_speed(speed);
}

int World::get_number_of_bodies()
{
    return this->bodies.size();
}

int World::get_number_of_revolute_joints(void)
{
    return this->revolute_joint_constraints.size();
}

std::optional<std::string> World::get_body_visual_shape_path(int id)
{
    return this->bodies[id].visual_object_path;
}

rs::Color World::get_body_color(int id)
{
    return this->bodies[id].color;
}

void World::set_body_color(int id, const rs::Color &color)
{
    this->bodies[id].color = color;
}

void World::set_body_color(int id, uint8_t r, uint8_t g, uint8_t b, uint8_t alpha)
{
    rs::Color color = {.r = r, .g = g, .b = b, .a = alpha};
    this->set_body_color(id, color);
}

void World::set_body_visual_shape_path(int id, std::string path)
{
    this->bodies[id].set_visual_object_path(path);
}
