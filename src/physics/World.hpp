#pragma once
#include "physics/math/math.hpp"
#include "bodies/Body.hpp"
#include "constraints/PositionalConstraint.hpp"
#include "constraints/RotationalConstraint.hpp"
#include "constraints/RevoluteJointConstraint.hpp"
#include "constraints/ContactConstraint.hpp"
#include "collisions/broad_phase.hpp"
#include "collisions/collisions.hpp"
#include <vector>
#include <tuple>
#include <map>
#include "aabb_tree.hpp"
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
        std::vector<ContactConstraint> contact_contraints;

        std::map<std::pair<int, int>, int> body_pair_to_contact_constraint_map;

        // Collision 
        std::vector<AABB> bodies_aabbs;
        std::vector<std::tuple<int, int, bool>> broad_phase_detections;
        AABBTree aabb_tree = AABBTree();

        // Planes
        // Static body used to solve the plane contact constraints
        int plane_body_idx = -1; // Default (-1) (you wont get anything!)


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
                        BodyType type = BodyType::DYNAMIC);
        void set_body_box_collider(int id, vec3 half_extents);
        void set_body_sphere_collider(int id, scalar radius);
        void set_body_capsule_collider(int id, scalar radius, scalar height);
        void set_body_plane_collider(int id, vec3 normal, scalar offset);
        int get_number_of_bodies();

        // Body info
        vec3 get_body_position(int id);
        quat get_body_orientation(int id);
        vec3 get_body_angular_velocity(int id);
        ShapeInfo get_collider_info(int id);
        AABB get_aabb(int id);

        // Collisions
        void collisions_detection_preparations(void);
        void broad_phase_collision_detection(void);
        void narrow_phase_collisions(scalar inverse_timestep);
        void narrow_phase_collisions_velocity_level(scalar timestep);

        /*
        * Creates a contact contraint between two bodies
        */
        int create_contact_constraint(int body_1_id, int body_2_id);

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


        // Functions for adding planes and heightmaps (These should be process separately)
        int add_plane(vec3 normal, scalar offset);
        std::tuple<bool,int, Plane> get_plane_info(void);
    };
} // namespace robosim