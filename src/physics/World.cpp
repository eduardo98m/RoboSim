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

    for (int i = 0; i < this->substeps; i++)
    {
        // this->.narrow_phase_collision()
        this->update_bodies_position_and_orientation(h);
        this->solve_positions(inv_h);
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

void World::solve_positions(scalar inv_h)
{
    for (PositionalConstraint constraint : this->positional_constraints)
    {
        constraint.apply_constraint(inv_h);
    }
    for (RotationalConstraint constraint : this->rotational_constraints)
    {
        constraint.apply_constraint(inv_h);
    }

    for (RevoluteJointConstraint constraint : this->revolute_joint_constraints)
    {
        constraint.apply_constraint(inv_h);
    }
}
void World::solve_velocities(scalar h) {
    for (RevoluteJointConstraint constraint : this->revolute_joint_constraints)
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
                                       scalar upper_limit)
{

    RevoluteJointConstraint constraint = RevoluteJointConstraint(&this->bodies[body_1_id],
                                                                 &this->bodies[body_2_id],
                                                                 ti::normalize(aligned_axis),
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