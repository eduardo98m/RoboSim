#pragma once
// Internal imports
#include "physics/math/math.hpp"
#include "bodies/Body.hpp"
#include "constraints/PositionalConstraint.hpp"
#include "constraints/RotationalConstraint.hpp"
#include "constraints/RevoluteJointConstraint.hpp"
#include "constraints/PrismaticJointConstraint.hpp"
#include "constraints/FixedJointConstraint.hpp"
#include "constraints/ContactConstraint.hpp"
#include "collisions/broad_phase.hpp"
#include "collisions/collisions.hpp"
#include "collisions/collider.hpp"
#include "physics/articulatedSystem/ArticulatedSystem.hpp"
#include "aabb_tree.hpp"
#include "hpp/fcl/broadphase/broadphase_collision_manager.h"
#include "visualShapes/VisualShape.hpp"

// Standart library
#include <vector>
#include <tuple>
#include <map>
#include <memory>
#include <limits>
#include <optional>

// HPP-FCL (Human path planner fast collision library)
#include <hpp/fcl/broadphase/broadphase_dynamic_AABB_tree_array.h>
#include <hpp/fcl/broadphase/default_broadphase_callbacks.h>

// URDF loading library
#include "urdf/model.h"
#include "urdf/link.h"
#include "urdf/joint.h"

// Create a blank class called World
namespace robosim
{
    // Maximum number of collision groups (adjust as needed)
    const int MAX_COLLISION_GROUPS = 32;

    class World
    {
    private:
        // time-steps
        scalar timestep;
        // number of sub-steps
        int substeps;
        // Vector of bodies of the world
        std::vector<std::shared_ptr<Body>> bodies;
        // Vector of colliders
        std::vector<Collider> colliders;
        // Vector of visual shapes
        std::vector<VisualShape> visual_shapes;
        // Vector of positional constraints
        std::vector<std::shared_ptr<PositionalConstraint>> positional_constraints;
        // Vector of rotational constraints
        std::vector<std::shared_ptr<RotationalConstraint>> rotational_constraints;
        // Vector of revolute joint constraints
        std::vector<std::shared_ptr<RevoluteJointConstraint>> revolute_joint_constraints;
        // Vector of prismatic joint constraints
        std::vector<std::shared_ptr<PrismaticJointConstraint>> prismatic_joint_constraints;
        // Vector of jixed joint constraints
        std::vector<std::shared_ptr<FixedJointConstraint>> fixed_joint_constraints;
        
        // Collision groups
        // Unordered map that maps the bodies to collision groups
        std::unordered_map<size_t, uint32_t> collision_groups;
        // Adjacent links collision filter
        std::map<std::pair<size_t, size_t>, bool> adajacent_links_filter;
        // Collision pairs from the aabb
        std::vector<std::pair<size_t, size_t>> potential_collision_pairs;
        // Articulated systems
        std::vector<ArticulatedSystem> articulated_systems;
        
        /**
         * Updates the positions and orientations of all bodies in the world based on their velocities and the given time step.
         *
         * @param h The time step (substep).
         */
        void update_bodies_position_and_orientation(scalar h);

        /**
         * Solves the positional constraints (e.g., contact, positional, rotational, and revolute joint constraints) for all bodies in the world.
         *
         * @param inv_h The inverse of the time step (substep).
         * @param h The time step (substep).
         */
        void solve_positions(scalar inv_h, scalar h);
        
        /**
         * Updates the linear and angular velocities of all bodies in the world based on their positions and orientations.
         *
         * @param inv_h The inverse of the time step.
         */
        void update_bodies_velocities(scalar inv_h);

        /**
         * Solves the velocity constraints (e.g., contact constraints and revolute joint damping) for all bodies in the world.
         *
         * @param h The time step.
         */
        void solve_velocities(scalar inv_h);

    public:
        // AABB Collision Tree
        AABBTree aabb_tree;
        // Contact constraints
        std::vector<ContactConstraint> contact_constraints;
        // Constructor
        /**
         * World constructor.
         *
         * @param timestep The time step for the simulation.
         * @param substeps The number of substeps per time step.
         */
        World(scalar timestep = 0.01, int substeps = 20);
        // Destructor
        //~World();

        /**
         * Performs a single simulation step.
         */
        void step();

        /**
         * Gets the time step of the world.
         *
         * @return The time step.
         */
        scalar get_time_step();

        /**
         * Sets the gravity acceleration for all bodies in the world.
         *
         * @param gravity The gravity acceleration vector.
         */
        void set_gravity(vec3 gravity);

        /**
         * Adds a body to the world.
         *
         * @param body The body to add.
         * @return The index of the added body.
         */
        size_t add_body(std::shared_ptr<Body> body);

        /**
         * @brief Creates a new body in the world.
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
        size_t create_body(vec3 position = vec3(0.0, 0.0, 0.0),
                        quat orientation = quat(1.0, 0.0, 0.0, 0.0),
                        vec3 linear_velocity = vec3(0.0, 0.0, 0.0),
                        vec3 angular_velocity = vec3(0.0, 0.0, 0.0),
                        scalar mass = 1.0,
                        mat3 inertia_tensor = mat3(1.0, 0.0, 0.0,
                                                   0.0, 1.0, 0.0,
                                                   0.0, 0.0, 1.0),
                        BodyType type = BodyType::DYNAMIC);
        
        
        std::string get_body_info_str(int id);

        /**
         * Helper function to add colliders
         */
        size_t add_collider(Collider collider);
        
        /**
         * Sets a box collider for a body in the world.
         *
         * @param id The index of the body.
         * @param half_extents The half extents of the box (i.e., half the dimensions along each axis).
         */
        size_t attach_box_collider(size_t id,
                                   vec3 half_extents,
                                   vec3 position = {0.0, 0.0, 0.0},
                                   quat orientation = {1.0, 0.0, 0.0, 0.0},
                                   scalar restitution = 0.5,
                                   scalar dynamic_friction = 0.5,
                                   scalar static_friction = 0.5,
                                   bool update_inertia = true);
        /**
         * Sets a sphere collider for a body in the world.
         *
         * @param id The index of the body.
         * @param radius The radius of the sphere.
         */
        size_t attach_sphere_collider(size_t id,
                                      scalar radius,
                                      vec3 position = {0.0, 0.0, 0.0},
                                      quat orientation = {1.0, 0.0, 0.0, 0.0},
                                      scalar restitution = 0.5,
                                      scalar dynamic_friction = 0.5,
                                      scalar static_friction = 0.5,
                                      bool update_inertia = true);

        /**
         * Sets a capsule collider for a body in the world. (NOT Implemented yet)
         *
         * @param id The index of the body.
         * @param radius The radius of the capsule.
         * @param height The height of the capsule.
         */
        size_t attach_capsule_collider(size_t id,
                                       scalar radius,
                                       scalar height,
                                       vec3 position = {0.0, 0.0, 0.0},
                                       quat orientation = {1.0, 0.0, 0.0, 0.0},
                                       scalar restitution = 0.5,
                                       scalar dynamic_friction = 0.5,
                                       scalar static_friction = 0.5,
                                       bool update_inertia = true);

        /**
         * Sets a cylinder collider for a body in the world.
         *
         * @param id The index of the body.
         * @param radius The radius of the cylinder.
         * @param height The height of the cylinder.
         */
        size_t attach_cylinder_collider(size_t id,
                                        scalar radius,
                                        scalar height,
                                        vec3 position = {0.0, 0.0, 0.0},
                                        quat orientation = {1.0, 0.0, 0.0, 0.0},
                                        scalar restitution = 0.5,
                                        scalar dynamic_friction = 0.5,
                                        scalar static_friction = 0.5,
                                        bool update_inertia = true);

        /**
         * Sets a plane collider for a body in the world.
         *
         * @param id The index of the body.
         * @param normal The normal vector of the plane.
         * @param offset The offset of the plane from the origin.
         */
        size_t attach_plane_collider(size_t id,
                                     vec3 normal,
                                     scalar offset,
                                     scalar restitution = 0.5,
                                     scalar dynamic_friction = 0.5,
                                     scalar static_friction = 0.5);

        /**
         * @brief Sets a heightmap collider for a body in the world.
         *
         */
        size_t attach_heightmap_collider(size_t id,
                                         scalar x_scale,
                                         scalar y_scale,
                                         std::vector<scalar> heightdata,
                                         size_t x_dims,
                                         size_t y_dims,
                                         scalar restitution = 0.5,
                                         scalar dynamic_friction = 0.5,
                                         scalar static_friction = 0.5);

        /**
         * @brief Recalculate the inertia matrix of a body based on the attached colliders volumes.
         * 
         * @param body_id The unique id of the body you want to recalculate the inertia matrix.
         */
        void recalculate_inertia_with_colliders(size_t body_id);

        /**
         * 
        */
        bool check_aabb_collision(const Collider &col_1, const Body &body_1, const Collider &col_2, const Body &body_2);

        void narrow_phase_collision_detection_and_response(scalar inverse_time_step);

        /**
         * Gets the number of bodies in the world.
         *
         * @return The number of bodies.
         */
        int get_number_of_bodies();

        /**
         * Gets the number of collideres in the world.
         *
         * @return The number of collideres.
         */
        int get_number_of_colliders(void);

        /**
         * Gets the position of a body in the world.
         *
         * @param id The index of the body.
         * @return The position of the body.
         */
        vec3 get_body_position(int id);

        /**
         * @brief REturns the orientation (as a quaternion) of a body given its `id`
         *
         * @param id The index of the body.
         * @return The orientation of the body.
         */
        quat get_body_orientation(int id);

        
        /**
         * @brief Returns the linear velocity (as a vec3) of a body given its `id`
         *
         * @param id The index of the body.
         * @return The velocity (in world frame)
         */
        vec3 get_body_velocity(int id);

        /**
         * @brief Gets the angular velocity of a body in the world.
         *
         * @param id The index of the body.
         * @return The angular velocity of the body.
         */
        vec3 get_body_angular_velocity(int id);

        /**
         * @brief Gets the path to the visual shape of a body in the world.
         *
         * @param id The index of the body.
         * @return The path to the visual shape of the body, or nullopt if not set.
         */
        std::optional<std::string> get_visual_shape_path(int id);

        /**
         * @brief Sets the color of a body in the world.
         *
         * @param id The index of the body.
         * @param r The red component of the color (0-255).
         * @param g The green component of the color (0-255).
         * @param b The blue component of the color (0-255).
         * @param alpha The alpha component of the color (0-255).
         **/
        void set_visual_shape_color(int id, uint8_t r, uint8_t g, uint8_t b, uint8_t alpha = 255);
        
        /**
         * @brief Sets the color of a body in the world.
         *
         * @param id The index of the body.
         * @param color The color to set.
         */
        void set_visual_shape_color(int id, const rs::Color &color);

        /**
         * Sets the static fricction coefficient of a body
         *
         * @param id The index of the body.
         * @param coeff The static fricction coefficient of the body.
         **/
        void set_body_static_friccion_coefficient(int id, scalar coeff);

        /**
         * @brief Sets the dynamic fricction coefficient of a body
         *
         * @param id The index of the body.
         * @param coeff The dynamic fricction coefficient of the body.
         */
        void set_body_dynamic_friccion_coefficient(int id, scalar coeff);

        /**
         * @brief Sets the restitution coefficient of a body
         *
         * @param id The index of the body.
         * @param coeff The restitution coefficient of the body.
         */
        void set_body_restitution_coefficient(int id, scalar coeff);

        /**
         * @brief Returns the color of a body in the world.
         *
         * @param id The index of the body.
         * @return The color of the body.
         */
        rs::Color get_visual_shape_color(int id);

        /**
         * @brief Returns the geometry for a collider in the world.
         *
         * @param id The index of the collider.
         * @return A shared pointer to the geometry of the collider.
         */
        std::shared_ptr<hpp::fcl::CollisionGeometry> get_collider_geometry(int id);

        /**
         * @brief Gets the pose of a collider in world cooridnates.
         * @param id Unique id of the collider
         * @returns A pair of vector and quaternion
        */
        std::pair<vec3, quat> get_collider_pose(size_t id);
        
        /**
         * @brief Gets the axis-aligned bounding box (AABB) of a collider in the world.
         *
         * @param id The index of the collider.
         * @return The AABB of the collider.
         */
        AABB get_aabb(int id);

        /**
         * @brief Prepares the world for collision detection by creating contact constraints between potentially colliding bodies.
         */
        void collisions_detection_preparations(void);
        
        /**
         * @brief Performs broad-phase collision detection for all bodies in the world.
         * Currently is performed as an O(n^2) operation.
         */
        void broad_phase_collision_detection(void);

        /**
         * @brief Performs narrow-phase collision detection for all bodies in the world at the position level.
         *
         * @param inverse_timestep The inverse of the time step.
         */
        void narrow_phase_collisions(scalar inverse_timestep);

        /**
         * @brief Performs narrow-phase collision detection for all bodies in the world at the velocity level.
         *
         * @param timestep The time step.
         */
        void narrow_phase_collisions_velocity_level(scalar timestep);

        /**
         * @brief Performs a raycast in the world and returns the intersection points.
         * The performance of this function is still experimental (is not 100% bug free).
         *
         * @param start The start point of the ray.
         * @param end The end point of the ray.
         * @return A vector of intersection points.
         */
        std::vector<vec3> raycast(vec3 start, vec3 end);

        /**
         * @brief Performs a "raycast" in the world and returns the intersection points.
         * WARNING Not working properly
         *
         * @param center Center of the scan
         * @param radius Radius of the scan
         * @param axis Axis that will orient the scan
         * @return A vector of intersection points.
         */
        std::vector<vec3> disc_raycast(vec3 center, scalar radius, vec3 axis);

        /**
         * @brief Checks if two bodies can collide based on their collision groups.
         *
         * @param bodyA The index (id) of the first body.
         * @param bodyB The index (id) of the second body.
         * @return True if the bodies can collide, false otherwise.
         */
        bool can_collide(size_t bodyA, size_t bodyB) const;

        /**
         * @brief Sets the collision group for a body in the world.
         *
         * @param id The index of the body.
         * @param collision_group The collision group to set.
         */
        void set_collision_group(size_t id, u_int32_t collision_group);

        /**
         * @brief Creates a positional constraint between two bodies in the world.
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

        /**
         * @brief Creates a revolute joint constraint between two bodies in the world.
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
                                       JointControlType type = JointControlType::FREE,
                                       bool limited = false,
                                       scalar lower_limit = 0.0,
                                       scalar upper_limit = 0.0,
                                       bool set_limit_axis = false,
                                       vec3 limit_axis = {1.0, 0.0, 0.0});

        /**
         * @brief Gets the number of revolute joint constraints in the world.
         *
         * @return The number of revolute joint constraints.
         */
        int get_number_of_revolute_joints(void);
        
        /**
         * @brief Gets the number of prismatic joint constraints in the world.
         *
         * @return The number of prismatic joint constraints.
         */
        int get_number_of_prismatic_joints(void);

        /**
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

        /**
         * Adds a rotational constraint to the world.
         *
         * @param constraint The rotational constraint to add.
         * @return The index of the added rotational constraint.
         */
        int add_rotational_constraint(std::shared_ptr<RotationalConstraint> cosntraint);

        /**
         * Gets the information about a revolute joint constraint in the world.
         *
         * @param id The index of the revolute joint constraint.
         * @return The RevoluteJointInfo structure containing the joint information.
         */
        RevoluteJointInfo get_revolute_joint_info(int id);

        /**
         * @brief Gets the infoformation struct of a prismatic joint give its id
         * 
         * @param id Unique id of the prismatic joint
         * @returns The PrismatiJointInfo struc containing the prismatc joint data.
        */
        PrismaticJointInfo get_prismatic_joint_info(int id);


        /**
         * @brief Sets the target position of a prismatic joint conmstraint.
         * @param id Unique id of the prismatic joint
         * @param position Target position to set the prismatic joint to go to.
        */
        void set_prismatic_joint_target_position(int id, scalar position);

        /**
         * @brief Sets the target speed of a prismatic joint conmstraint.
         * @param id Unique id of the prismatic joint
         * @param speed Target speed to set the prismatic joint to.
        */
       void set_prismatic_joint_target_speed(int id, scalar speed);

        /**
         * Creates a fixed constraint between two bodies in the world.
         *
         * @param body_1_id The index of the first body.
         * @param body_2_id The index of the second body.
         * @param r_1 The constraint position relative to the first body (in the local coordinates of the first body).
         * @param r_2 The constraint position relative to the second body (in the local coordinates of the second body).
         * @return The index of the created rotational constraint.
         */
        int create_fixed_joint_constraint(int body_1_id,
                                         int body_2_id,
                                         vec3 r_1,
                                         vec3 r_2);

        /**
         * @brief Returns the information struct a the fixed joint given its id.
         * 
         * @param id The id oh the joint you want to get the info.
        */
        FixedJointInfo get_fixed_joint_info(int id);

        /**
         * Sets the target angle for a revolute joint constraint in the world.
         *
         * @param id The index of the revolute joint constraint.
         * @param angle The target angle to set.
         */
        void set_revolute_joint_target_angle(int id, scalar angle);

        /**
         * Sets the target speed for a revolute joint constraint in the world.
         *
         * @param id The index of the revolute joint constraint.
         * @param speed The target speed to set.
         */
        void set_revolute_joint_target_speed(int id, scalar speed);


        /**
         * Creates a revolute joint constraint between two bodies in the world.
         *
         * @param body_1_id The index of the first body.
         * @param body_2_id The index of the second body.
         * @param moving_axis The axis in which the joint moves.
         * @param r_1 The constraint position relative to the first body (in the local coordinates of the first body).
         * @param r_2 The constraint position relative to the second body (in the local coordinates of the second body).
         * @param compliance The compliance of the constraint.
         * @param damping The damping of the constraint.
         * @param type The type of the revolute joint (FREE, LIMITED_ANGLE, or SPEED_CONTROLLED).
         * @param limited Whether the joint has angle limits or not.
         * @param lower_limit The lower angle limit (if limited).
         * @param upper_limit The upper angle limit (if limited).
         * @return The index of the created prismatic joint constraint.
         */
        int create_prismatic_joint_constraint(int body_1_id,
                                       int body_2_id,
                                       vec3 moving_axis,
                                       vec3 r_1,
                                       vec3 r_2,
                                       scalar compliance = 0.0,
                                       scalar damping = 0.0,
                                       JointControlType type = JointControlType::FREE,
                                       bool limited = false,
                                       scalar lower_limit = 0.0,
                                       scalar upper_limit = 0.0);

        /**
         * Adds a plane to the world.
         *
         * @param normal The normal vector of the plane.
         * @param offset The offset of the plane from the origin.
         * @return The index of the body representing the plane.
         */
        int add_plane(vec3 normal, scalar offset);

        // Functions for the articulates system management

        /**
         * Groups a series of (connected) bodies and joints into an articulated system
         *
         * @param joint_ids The ids of the joints
         * @param joint_types The types of the joints (constraints)
         * @return The index of the articulated system
         */
        size_t create_articulated_system(
            const std::vector<size_t> &joint_ids,
            const std::vector<JointType> &joint_types,
            const std::vector<size_t> &link_ids);

        /**
         * Gets the state of the articulated system
         *
         * @param id The id of the articulated_system
         * @return A vector of scalars where:
         *         * The first 3 elements are the base_link positions
         *         * The next 4 elements are the base link orientation expreesed as a quaternion (q.x, q.y, q.z, q.w)
         *         * The following elementes are the articulation variables
         */
        std::vector<scalar> get_articulated_system_state(size_t id);

        /**
         * @brief Sets the joint targets of and articulated system joints.
         *
         * @param id The id of the articulated_system
         */
        void set_articulated_system_joint_targets(size_t id, std::vector<scalar> joint_targets);

        /**
         * Gets the pose of a link of an articulated sistem
         *
         * @param id The id of the articulated_system
         * @return The pose (position and orientation) of the selected link
         */
        pose get_articulated_system_link_pose(size_t id, size_t link_id);

        /**
         * Loads an urdf file into the robosim world and returns the articulated system id
         */
        size_t load_urdf(const std::string &filename, const vec3 &base_position, const quat &base_orientation);

        /**
         * @brief Adds a link from a urdf file to the world
         * 
         * @param link 
        */
        size_t add_urdf_link(const std::shared_ptr<urdf::Link> &link,
                             std::map<std::string, size_t> &link_name_to_body_id,
                             std::string filepath,
                             const vec3 *base_position = nullptr,
                             const quat *base_orientation = nullptr,
                             bool root_link = false);

        // Visual shapes
        /**
         * @brief Adds a visual shape to the visual shape vector
         * **/
        size_t add_visual_shape(VisualShape visual_shape);

        size_t get_number_of_visual_shapes(void);

        /**
         * @brief Attaches mesh to a body
         * @param id The id of the body
         * @param mesh_path The path to the mesh file (currently only .obj files)
         * **/
        size_t attach_mesh_visual_shape(size_t id, 
                                        std::string mesh_path,
                                        vec3 scale = {1.0, 1.0, 1.0},
                                        vec3 position = {0.0, 0.0, 0.0},
                                        quat orientation = {1.0, 0.0, 0.0, 0.0},
                                        rs::Color color = {.r = 51, .g = 103, .b = 59, .a = 255} ); 

        size_t attach_box_visual_shape(size_t id, vec3 half_extents,
                                       vec3 position = {0.0, 0.0, 0.0},
                                       quat orientation = {1.0, 0.0, 0.0, 0.0},
                                       rs::Color color = {.r = 208, .g = 83, .b = 83, .a = 255});

        size_t attach_sphere_visual_shape(size_t id, scalar radius,
                                          vec3 position = {0.0, 0.0, 0.0},
                                          quat orientation = {1.0, 0.0, 0.0, 0.0},
                                          rs::Color color = {.r = 249, .g = 231, .b = 132, .a = 255});

        size_t attach_capsule_visual_shape(size_t id, scalar radius, scalar height,
                                           vec3 position = {0.0, 0.0, 0.0},
                                           quat orientation = {1.0, 0.0, 0.0, 0.0},
                                           rs::Color color = {.r = 229, .g = 143, .b = 101, .a = 255});

        size_t attach_cylinder_visual_shape(size_t id, scalar radius, scalar height,
                                            vec3 position = {0.0, 0.0, 0.0},
                                            quat orientation = {1.0, 0.0, 0.0, 0.0},
                                            rs::Color color = {.r = 0, .g = 159, .b = 183, .a = 255});

        size_t attach_plane_visual_shape(size_t id, vec3 normal, scalar offset, rs::Color color = {.r = 125, .g = 195, .b = 158, .a = 255});

        size_t attach_heightmap_visual_shape(size_t id, scalar x_scale, scalar y_scale, std::vector<scalar> heightdata, size_t x_dims, size_t y_dims, rs::Color color = {.r = 26, .g = 131, .b = 121, .a = 255});

        std::pair<vec3, quat> get_visual_shape_pose(size_t id);

        /**
         * @brief returns a pointer to the geometry of the visual shape
        */
        std::shared_ptr<hpp::fcl::CollisionGeometry>  get_visual_shape_geometry(size_t id);

        /**
         * @brief returns the scaling vector of the visual shape 
        */
        vec3 get_visual_shape_scale(size_t id);
    };
} // namespace robosim