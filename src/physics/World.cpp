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
    for (ContactConstraint &constraint : this->contact_contraints)
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
    for (ContactConstraint &constraint : this->contact_contraints)
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
    for (ContactConstraint &constraint : this->contact_contraints)
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
    this->bodies[id].set_box_collider(half_extents);
}

void World::set_body_cylinder_collider(int id, scalar radius, scalar height)
{
    this->bodies[id].set_cylinder_collider(radius, height);
}

void World::set_body_sphere_collider(int id, scalar radius)
{
    this->bodies[id].set_sphere_collider(radius);
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

    this->contact_contraints.push_back(constraint);
    return (int)(this->contact_contraints.size() - 1);
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

    // vec3 expansion_factor = ti::abs(2.0 * this->bodies[id].linear_velocity * this->timestep);
    // aabb = AABB{
    //     .min = aabb.min - expansion_factor,
    //     .max = aabb.max + expansion_factor,
    // };
    
    return aabb;
}

void World::collisions_detection_preparations(void)
{

    int n_bodies = this->get_number_of_bodies();

    for (int i = 0; i < (n_bodies - 1); i++)
    {
        for (int j = i + 1; j < n_bodies; j++)
        {
            int  id = this->create_contact_constraint(i, j);
            body_pair_to_contact_constraint_map.insert({{i,j}, id});
        }
    }
}

// Collisions
void World::broad_phase_collision_detection(void)
{

    for (ContactConstraint &constraint : this->contact_contraints)
    {
        constraint.check_broad_phase(timestep);
    }
      
}

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
