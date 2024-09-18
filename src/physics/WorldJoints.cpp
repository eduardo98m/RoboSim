#include "World.hpp"
using namespace robosim;

int World::create_prismatic_joint_constraint(int body_1_id,
                                      int body_2_id,
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

    std::shared_ptr<PrismaticJointConstraint> constraint = std::make_shared<PrismaticJointConstraint>(
                                                                 this->bodies[body_1_id],
                                                                 this->bodies[body_2_id],
                                                                 moving_axis,
                                                                 r_1,
                                                                 r_2,
                                                                 compliance,
                                                                 damping,
                                                                 type,
                                                                 limited,
                                                                 lower_limit,
                                                                 upper_limit);

    this->prismatic_joint_constraints.push_back(constraint);
    return (int)(this->prismatic_joint_constraints.size() - 1);
}

// Revolute joints
PrismaticJointInfo World::get_prismatic_joint_info(int id)
{
    return this->prismatic_joint_constraints[id]->get_info();
}

void World::set_prismatic_joint_target_position(int id, scalar position)
{
    this->prismatic_joint_constraints[id]->set_traget_position(position);
}

void World::set_prismatic_joint_target_speed(int id, scalar speed)
{
    this->prismatic_joint_constraints[id]->set_target_speed(speed);
}