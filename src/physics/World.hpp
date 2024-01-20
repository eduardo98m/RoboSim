#pragma once
#include "physics/math/math.hpp"
#include "bodies/Body.hpp"
#include "constraints/PositionalConstraint.hpp"
#include "constraints/RotationalConstraint.hpp"
#include "constraints/RevoluteJointConstraint.hpp"
#include <vector>

// Create a blank class called World
namespace robosim
{
    class World
    {
    private:
        scalar timestep;
        int substeps;

        std::vector<Body> bodies;
        std::vector<PositionalConstraint> positional_constraints;
        std::vector<RotationalConstraint> rotational_constraints;
        std::vector<RevoluteJointConstraint> revolute_joint_constraints;

        void update_bodies_position_and_orientation(scalar h);
        void solve_positions(scalar inv_h, scalar h);
        void update_bodies_velocities(scalar inv_h);
        void solve_velocities(scalar inv_h);

    public:
        // Constructor
        World(scalar timestep = 0.01, int substeps = 20);
        // Destructor
        //~World();

        // Get the time step of the world
        scalar get_time_step();

        void set_gravity(vec3 gravity);

        int add_body(Body body);
        int create_body(vec3 position = vec3(0.0, 0.0, 0.0),
                        quat orientation = quat(1.0, 0.0, 0.0, 0.0),
                        vec3 linear_velocity = vec3(0.0, 0.0, 0.0),
                        vec3 angular_velocity = vec3(0.0, 0.0, 0.0),
                        scalar mass = 1.0,
                        mat3 inertia_tensor = mat3(1.0, 0.0, 0.0,
                                                   0.0, 1.0, 0.0,
                                                   0.0, 0.0, 1.0),
                        BodyType type = DYNAMIC);
        void set_body_box_collider(int id, vec3 half_extents);
        void set_body_sphere_collider(int id, scalar radius);
        void set_body_capsule_collider(int id, scalar radius, scalar height);
        int get_number_of_bodies();

        // Body info
        vec3 get_body_position(int id);
        quat get_body_orientation(int id);
        vec3 get_body_angular_velocity(int id);
        ShapeInfo get_collider_info(int id);

        //
        int create_positional_constraint(int body_1_id,
                                         int body_2_id,
                                         vec3 r_1,
                                         vec3 r_2,
                                         scalar compliance = 0.0,
                                         scalar damping = 0.0);

        int create_revolute_constraint(int body_1_id,
                                       int body_2_id,
                                       vec3 aligned_axis,
                                       vec3 r_1,
                                       vec3 r_2,
                                       scalar compliance = 0.0,
                                       scalar damping = 0.0,
                                       RevoluteJointType type = FREE,
                                       bool limited = false,
                                       scalar lower_limit = 0.0,
                                       scalar upper_limit = 0.0,
                                       bool set_limit_axis = false,
                                       vec3 limit_axis = {1.0, 0.0, 0.0});
        
        int get_number_of_revolute_joints(void);

        int create_rotational_constraint(int body_1_id,
                                         int body_2_id,
                                         vec3 r_1,
                                         vec3 r_2,
                                         scalar compliance = 0.0,
                                         scalar damping = 0.0);

        int add_rotational_constraint(RotationalConstraint cosntraint);

        void step();

        // Revolute joint functions
        RevoluteJointInfo get_revolute_joint_info(int id);
        void set_revolute_joint_target_angle(int id, scalar angle);
        void set_revolute_joint_target_speed(int id, scalar speed);

        // Getting Body info
    };
} // namespace robosim