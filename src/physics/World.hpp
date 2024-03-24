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
#include <memory>
#include "aabb_tree.hpp"
#include "hpp/fcl/broadphase/broadphase_collision_manager.h"
#include <hpp/fcl/broadphase/broadphase_dynamic_AABB_tree_array.h>
#include <hpp/fcl/broadphase/default_broadphase_callbacks.h>

// Create a blank class called World
namespace robosim
{
    // Maximum number of collision groups (adjust as needed)
    const int MAX_COLLISION_GROUPS = 32;

    class World
    {
    private:
        scalar timestep;
        int substeps;

        // Vector of bodies of the world
        std::vector<Body> bodies;
        // Vector of positional constraints
        std::vector<PositionalConstraint> positional_constraints;
        // Vector of rotational constraints
        std::vector<RotationalConstraint> rotational_constraints;
        // // Vector of revolute joint constraints
        std::vector<RevoluteJointConstraint> revolute_joint_constraints;
        // Map of body pairs to cotnact constraint
        std::map<std::pair<int, int>, int> body_pair_to_contact_constraint_map;
        // Collision groups
        // Unordered map that maps the bodies to collision groups
        std::unordered_map<size_t, uint32_t> collision_groups;

        // Plane (if used has a reserved attribute in the world class)
        //int plane_body_idx = -1; // Default (-1) (you wont get anything!)

        /*
         * Updates the positions and orientations of all bodies in the world based on their velocities and the given time step.
         *
         * @param h The time step (substep).
         */
        void update_bodies_position_and_orientation(scalar h);

        /*
         * Solves the positional constraints (e.g., contact, positional, rotational, and revolute joint constraints) for all bodies in the world.
         *
         * @param inv_h The inverse of the time step (substep).
         * @param h The time step (substep).
         */
        void solve_positions(scalar inv_h, scalar h);
        /*
         * Updates the linear and angular velocities of all bodies in the world based on their positions and orientations.
         *
         * @param inv_h The inverse of the time step.
         */
        void update_bodies_velocities(scalar inv_h);

        /*
         * Solves the velocity constraints (e.g., contact constraints and revolute joint damping) for all bodies in the world.
         *
         * @param h The time step.
         */
        void solve_velocities(scalar inv_h);

    public:
        // Contact constraints
        std::vector<ContactConstraint> contact_constraints;
        // Constructor
        /*
         * World constructor.
         *
         * @param timestep The time step for the simulation.
         * @param substeps The number of substeps per time step.
         */
        World(scalar timestep = 0.01, int substeps = 20);
        // Destructor
        //~World();

        /*
         * Performs a single simulation step.
         */
        void step();

        // Get the time step of the world
        /*
         * Gets the time step of the world.
         *
         * @return The time step.
         */
        scalar get_time_step();

        /*
         * Sets the gravity acceleration for all bodies in the world.
         *
         * @param gravity The gravity acceleration vector.
         */
        void set_gravity(vec3 gravity);

        /*
         * Adds a body to the world.
         *
         * @param body The body to add.
         * @return The index of the added body.
         */
        int add_body(Body body);

        /*
         * Creates a new body in the world.
         *
         * @param position The initial position of the body.
         * @param orientation The initial orientation of the body.
         * @param linear_velocity The initial linear velocity of the body.
         * @param angular_velocity The initial angular velocity of the body.
         * @param mass The mass of the body.
         * @param inertia_tensor The inertia tensor of the body (In local coordiantes).
         * @param type The type of the body (STATIC or DYNAMIC).
         * @return The index of the created body.
         */
        int create_body(vec3 position = vec3(0.0, 0.0, 0.0),
                        quat orientation = quat(1.0, 0.0, 0.0, 0.0),
                        vec3 linear_velocity = vec3(0.0, 0.0, 0.0),
                        vec3 angular_velocity = vec3(0.0, 0.0, 0.0),
                        scalar mass = 1.0,
                        mat3 inertia_tensor = mat3(1.0, 0.0, 0.0,
                                                   0.0, 1.0, 0.0,
                                                   0.0, 0.0, 1.0),
                        BodyType type = BodyType::DYNAMIC);
        /*
         * Sets a box collider for a body in the world.
         *
         * @param id The index of the body.
         * @param half_extents The half extents of the box (i.e., half the dimensions along each axis).
         */
        void set_body_box_collider(int id, vec3 half_extents);
        /*
         * Sets a sphere collider for a body in the world.
         *
         * @param id The index of the body.
         * @param radius The radius of the sphere.
         */
        void set_body_sphere_collider(int id, scalar radius);

        /*
         * Sets a capsule collider for a body in the world.
         *
         * @param id The index of the body.
         * @param radius The radius of the capsule.
         * @param height The height of the capsule.
         */

        void set_body_capsule_collider(int id, scalar radius, scalar height);

        /*
         * Sets a cylinder collider for a body in the world.
         *
         * @param id The index of the body.
         * @param radius The radius of the cylinder.
         * @param height The height of the cylinder.
         */
        void set_body_cylinder_collider(int id, scalar radius, scalar height);

        /*
         * Sets a plane collider for a body in the world.
         *
         * @param id The index of the body.
         * @param normal The normal vector of the plane.
         * @param offset The offset of the plane from the origin.
         */
        void set_body_plane_collider(int id, vec3 normal, scalar offset);

        /*
         * Gets the number of bodies in the world.
         *
         * @return The number of bodies.
         */
        int get_number_of_bodies();

        /*
         * Gets the position of a body in the world.
         *
         * @param id The index of the body.
         * @return The position of the body.
         */
        vec3 get_body_position(int id);

        /*
         * Gets the orientation of a body in the world.
         *
         * @param id The index of the body.
         * @return The orientation of the body.
         */
        quat get_body_orientation(int id);

        /*
         * Gets the angular velocity of a body in the world.
         *
         * @param id The index of the body.
         * @return The angular velocity of the body.
         */
        vec3 get_body_angular_velocity(int id);

        /*
         * Gets the path to the visual shape of a body in the world.
         *
         * @param id The index of the body.
         * @return The path to the visual shape of the body, or nullopt if not set.
         */
        std::optional<std::string> get_body_visual_shape_path(int id);
        /*
         * Sets the path to the visual shape of a body in the world.
         *
         * @param id The index of the body.
         * @param path The path to the visual shape.
         */
        void set_body_visual_shape_path(int id, std::string path);
        /*
         * Sets the color of a body in the world.
         *
         * @param id The index of the body.
         * @param r The red component of the color (0-255).
         * @param g The green component of the color (0-255).
         * @param b The blue component of the color (0-255).
         * @param alpha The alpha component of the color (0-255).
         */
        void set_body_color(int id, uint8_t r, uint8_t g, uint8_t b, uint8_t alpha = 255);
        /*
         * Sets the color of a body in the world.
         *
         * @param id The index of the body.
         * @param color The color to set.
         */
        void set_body_color(int id, const rs::Color &color);

        /*
         * Gets the color of a body in the world.
         *
         * @param id The index of the body.
         * @return The color of the body.
         */
        rs::Color get_body_color(int id);

        /*
         * Gets the collider information for a body in the world.
         *
         * @param id The index of the body.
         * @return A shared pointer to the collider geometry of the body.
         */
        std::shared_ptr<hpp::fcl::CollisionGeometry> get_collider_info(int id);

        /*
         * Gets the axis-aligned bounding box (AABB) of a body in the world.
         *
         * @param id The index of the body.
         * @return The AABB of the body.
         */
        AABB get_aabb(int id);

        // Collisions

        /*
         * Prepares the world for collision detection by creating contact constraints between potentially colliding bodies.
         */
        void collisions_detection_preparations(void);
        /*
         * Performs broad-phase collision detection for all bodies in the world.
         * Currently is performed as an O(n^2) operation.
         */
        void broad_phase_collision_detection(void);

        /*
         * Performs narrow-phase collision detection for all bodies in the world at the position level.
         *
         * @param inverse_timestep The inverse of the time step.
         */
        void narrow_phase_collisions(scalar inverse_timestep);

        /*
         * Performs narrow-phase collision detection for all bodies in the world at the velocity level.
         *
         * @param timestep The time step.
         */
        void narrow_phase_collisions_velocity_level(scalar timestep);

        /*
         * Performs a raycast in the world and returns the intersection points.
         * The performance of this function is still experimental (is not 100% bug free).
         *
         * @param start The start point of the ray.
         * @param end The end point of the ray.
         * @return A vector of intersection points.
         */
        std::vector<vec3> raycast(vec3 start, vec3 end);

        /*
         * Performs a "raycast" in the world and returns the intersection points.
         * Not really working properly
         *
         * @param center Center of the scan
         * @param radius Radius of the scan
         * @param axis Axis that will orient the scan
         * @return A vector of intersection points.
         */
        std::vector<vec3> disc_raycast(vec3 center, scalar radius, vec3 axis);

        /*
         * Checks if two bodies can collide based on their collision groups.
         *
         * @param bodyA The index (id) of the first body.
         * @param bodyB The index (id) of the second body.
         * @return True if the bodies can collide, false otherwise.
         */
        bool can_collide(size_t bodyA, size_t bodyB) const;

        /*
         * Sets the collision group for a body in the world.
         *
         * @param id The index of the body.
         * @param collision_group The collision group to set.
         */
        void set_collision_group(size_t id, u_int32_t collision_group);

        /*
         * Creates a contact constraint between two bodies in the world.
         *
         * @param body_1_id The index of the first body.
         * @param body_2_id The index of the second body.
         * @return The index of the created contact constraint.
         */
        int create_contact_constraint(int body_1_id, int body_2_id);

        /*
         * Creates a positional constraint between two bodies in the world.
         *
         * @param body_1_id The index of the first body.
         * @param body_2_id The index of the second body.
         * @param r_1 The constraint position relative to the first body (in the local coordinates of the first body).
         * @param r_2 The constraint position relative to the second body (in the local coordinates of the second body).
         * @param compliance The compliance of the constraint.
         * @param damping The damping of the constraint.
         * @return The index of the created positional constraint.
         */
        int create_positional_constraint(int body_1_id,
                                         int body_2_id,
                                         vec3 r_1,
                                         vec3 r_2,
                                         scalar compliance = 0.0,
                                         scalar damping = 0.0);

        /*
         * Creates a revolute joint constraint between two bodies in the world.
         *
         * @param body_1_id The index of the first body.
         * @param body_2_id The index of the second body.
         * @param aligned_axis The axis around which the joint rotates.
         * @param r_1 The constraint position relative to the first body (in the local coordinates of the first body).
         * @param r_2 The constraint position relative to the second body (in the local coordinates of the second body).
         * @param compliance The compliance of the constraint.
         * @param damping The damping of the constraint.
         * @param type The type of the revolute joint (FREE, LIMITED_ANGLE, or SPEED_CONTROLLED).
         * @param limited Whether the joint has angle limits or not.
         * @param lower_limit The lower angle limit (if limited).
         * @param upper_limit The upper angle limit (if limited).
         * @param set_limit_axis Whether to set the limit axis manually or not.
         * @param limit_axis The limit axis (if set_limit_axis is true).
         * @return The index of the created revolute joint constraint.
         */
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

        /*
         * Gets the number of revolute joint constraints in the world.
         *
         * @return The number of revolute joint constraints.
         */
        int get_number_of_revolute_joints(void);

        /*
         * Creates a rotational constraint between two bodies in the world.
         *
         * @param body_1_id The index of the first body.
         * @param body_2_id The index of the second body.
         * @param r_1 The constraint position relative to the first body (in the local coordinates of the first body).
         * @param r_2 The constraint position relative to the second body (in the local coordinates of the second body).
         * @param compliance The compliance of the constraint.
         * @param damping The damping of the constraint.
         * @return The index of the created rotational constraint.
         */
        int create_rotational_constraint(int body_1_id,
                                         int body_2_id,
                                         vec3 r_1,
                                         vec3 r_2,
                                         scalar compliance = 0.0,
                                         scalar damping = 0.0);

        /*
         * Adds a rotational constraint to the world.
         *
         * @param constraint The rotational constraint to add.
         * @return The index of the added rotational constraint.
         */
        int add_rotational_constraint(RotationalConstraint cosntraint);

        /*
         * Gets the information about a revolute joint constraint in the world.
         *
         * @param id The index of the revolute joint constraint.
         * @return The RevoluteJointInfo structure containing the joint information.
         */
        RevoluteJointInfo get_revolute_joint_info(int id);

        /*
         * Sets the target angle for a revolute joint constraint in the world.
         *
         * @param id The index of the revolute joint constraint.
         * @param angle The target angle to set.
         */
        void set_revolute_joint_target_angle(int id, scalar angle);

        /*
         * Sets the target speed for a revolute joint constraint in the world.
         *
         * @param id The index of the revolute joint constraint.
         * @param speed The target speed to set.
         */
        void set_revolute_joint_target_speed(int id, scalar speed);

        /*
         * Adds a plane to the world.
         *
         * @param normal The normal vector of the plane.
         * @param offset The offset of the plane from the origin.
         * @return The index of the body representing the plane.
         */
        int add_plane(vec3 normal, scalar offset);
    };
} // namespace robosim