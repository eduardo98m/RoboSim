#pragma once
#include "physics/math/math.hpp"
#include "Shapes.hpp"
#include "hpp/fcl/BVH/BVH_model.h"
#include "hpp/fcl/collision.h"
#include "hpp/fcl/collision_data.h"
#include <memory>
#include <optional>

enum BodyType
{
    STATIC,
    DYNAMIC
};

// TODO : Change this to another file
namespace rs
{
    struct Color
    {
        u_int8_t r = 255;
        u_int8_t g = 255;
        u_int8_t b = 255;
        u_int8_t a = 255;

        // Overloading the subscript operator []
        uint8_t &operator[](size_t index)
        {
            // Assuming index is within bounds (0 to 3)
            switch (index)
            {
            case 0:
                return r;
            case 1:
                return g;
            case 2:
                return b;
            case 3:
                return a;
            default:
                throw std::out_of_range("Index out of range");
            }
        }

        // Overloading const version of the subscript operator []
        const uint8_t &operator[](size_t index) const
        {
            // Assuming index is within bounds (0 to 3)
            switch (index)
            {
            case 0:
                return r;
            case 1:
                return g;
            case 2:
                return b;
            case 3:
                return a;
            default:
                throw std::out_of_range("Index out of range");
            }
        }
    };

};

class Body
{
private:
    /* linear and angular force */
    vec3 force = {0.0, 0.0, 0.0};
    vec3 torque = {0.0, 0.0, 0.0};

    /* Mass properties */
    // Mass of the body
    scalar mass;
    // Inverse mass of the body
    scalar inverse_mass;
    // Inertia tensor (In local coordinates)
    mat3 inertia_tensor;
    // Inverse inertia tensor (In local coordinates)
    mat3 inverse_inertia_tensor;
    // Inertia tensor in world cooridantes 
    mat3 inertia_tensor_world;
    // Inverse inertia tensor in world cooridantes 
    mat3 inverse_inertia_tensor_world;
    // Collision group. Collision groups function with bitwise operations 
    // (To learn more please check this stack overflow tread: 
    // https://stackoverflow.com/questions/39063949/cant-understand-how-collision-bit-mask-works)
    int collision_group;

public:
    // Type of the body
    BodyType type;
    // Position of the body. (In world coordinates)
    vec3 position;
    // Orientation of the body as a quaternion
    quat orientation;
    // Linear velocity of the body
    vec3 linear_velocity;
    // Angular velocity of the body
    vec3 angular_velocity;
    // Previous position of the body (at the previous substep)
    vec3 prev_position;
    // Previous orienatation of the body (at the previous substep)
    quat prev_orientation;
    // Previous linear velocity of the body (at the previous substep)
    vec3 prev_linear_velocity;
    // Previous angular velocity of the body (at the previous substep)
    vec3 prev_angular_velocity;
    // Collider information -> shared pointer to hpp::fcl::CollisionGeometry
    std::shared_ptr<hpp::fcl::CollisionGeometry> collider_info;

    // Optional string to the path of the visual object (i.e. visual shape) attached to the body
    std::optional<std::string> visual_object_path = std::nullopt;

    // Color of the body
    rs::Color color = {.r = 255, .g = 95, .b = 31, .a = 255};
    /* linear and angular velocity */
    

    /* Material properties */
    scalar dynamic_fricction_coeff = 0.5;
    scalar static_fricction_coeff = 0.5;

    /*
     * Class constructor
     *
     * @param position The initial position of the body
     * @param orientation The initial orientation of the body
     * @param linear_velocity The initial linear velocity of the body
     * @param angular_velocity The initial angular velocity of the body
     * @param mass The mass of the body
     * @param inertia_tensor The inertia tensor of the body
     * @param type The type of the body (BodyType::STATIC or BodyType::DYNAMIC)
     */
    Body(
        vec3 position = vec3(0.0, 0.0, 0.0),
        quat orientation = quat(1.0, 0.0, 0.0, 0.0),
        vec3 linear_velocity = vec3(0.0, 0.0, 0.0),
        vec3 angular_velocity = vec3(0.0, 0.0, 0.0),
        scalar mass = 1.0,
        mat3 inertia_tensor = mat3(1.0, 0.0, 0.0,
                                   0.0, 1.0, 0.0,
                                   0.0, 0.0, 1.0),
        BodyType type = BodyType::DYNAMIC);

    /*
     * Gets the positional generalized inverse mass of the body at a given position and direction
     *
     * @param r The position relative to the body's center of mass (in world coordinates)
     * @param n The direction vector (in world coordinates) (must be a unit vector)
     * @return The positional generalized inverse mass
     */
    scalar get_positional_generalized_inverse_mass(vec3 r, vec3 n);

    /*
     * Gets the rotational generalized inverse mass of the body for a given direction
     *
     * @param n The direction vector (in world coordinates) (must be a unit vector)
     * @return The rotational generalized inverse mass
     */
    scalar get_rotational_generalized_inverse_mass(vec3 n);

    /*
     * Updates the body's inertia tensor in world coordinates
     */
    void update_inertia_tensor_world();

    /*
     * Updates the body's position and orientation based on the current velocities and forces
     *
     * @param time_step The time step
     */
    void update_position_and_orientation(scalar time_step);

    /*
     * Updates the body's linear and angular velocities based on the current position and orientation.
     *
     * @param inverse_time_step The inverse of the time step
     */
    void update_velocities(scalar inverse_time_step);

    /*
     * Sets the gravity acceleration acting on the body
     *
     * @param gravity The gravity acceleration vector
     */
    void set_gravity(vec3 gravity);

    /*
     * Applies a force to the body (TODO: FIX THIS// make it work with local and world coordinates)
     * @param force The force that will be applied onto the body (In world coordinates)
     */
    void apply_force(vec3 force);

    /*
     * Applies a torque over the body
     * @param torque The torque vector (In world coordinates).
     */
    void apply_torque(vec3 torque);

    /*
     * Applies a positional constraint impulse to the body (updating its position and orientation).
     *
     * @param impulse The impulse vector
     * @param r The position relative to the body's center of mass (in world coordinates)
     */
    void apply_positional_constraint_impulse(vec3 impulse, vec3 r);

    /*
     * Applies a rotational constraint impulse to the body.(Updating its orientation)
     *
     * @param impulse The impulse vector
     */
    void apply_rotational_constraint_impulse(vec3 impulse);

    /*
     * Applies a positional velocity constraint impulse to the body. (Updating its velocity)
     *
     * @param impulse The impulse vector
     * @param r The position relative to the body's center of mass (in world coordinates)
     */
    void apply_positional_velocity_constraint_impulse(vec3 impulse, vec3 r);

    /*
     * Sets a box collider for the body
     *
     * @param half_extents The half extents of the box (i.e., half the dimensions along each axis)
     * @param recompute_inertia Whether to recompute the inertia tensor based on the collider shape
     */
    void set_box_collider(vec3 half_extents, bool recompute_inertia = true);

    /*
     * Sets a sphere collider for the body
     *
     * @param radius The radius of the sphere
     * @param recompute_inertia Whether to recompute the inertia tensor based on the collider shape
     */
    void set_sphere_collider(scalar radius, bool recompute_inertia = true);

    /*
     * Sets a capsule collider for the body
     *
     * @param radius The radius of the capsule
     * @param height The height of the capsule
     * @param recompute_inertia Whether to recompute the inertia tensor based on the collider shape
     */
    void set_capsule_collider(scalar radius, scalar height, bool recompute_inertia = true);

    /*
     * Sets a cylinder collider for the body
     *
     * @param radius The radius of the cylinder
     * @param height The height of the cylinder
     * @param recompute_inertia Whether to recompute the inertia tensor based on the collider shape
     */
    void set_cylinder_collider(scalar radius, scalar height, bool recompute_inertia = true);

    /*
     * Sets a plane collider for the body
     *
     * @param normal The normal vector of the plane
     * @param offset The offset of the plane from the origin
     */
    void set_plane_collider(vec3 normal, scalar offset);

    /*
     * Sets the inertia tensor of the body
     *
     * @param inertia_tensor The new inertia tensor
     */
    void set_intertia_tensor(const mat3 &intertia_tensor);

    /*
     * Sets the path to a visual object for the body
     *
     * @param path The path to the visual object file. (.obj file)
     */
    void set_visual_object_path(std::string path);
};