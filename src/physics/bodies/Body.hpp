#pragma once
#include "physics/math/math.hpp"
#include "Shapes.hpp"

// #include "hpp/fcl/math/transform.h"
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
    scalar mass;
    scalar inverse_mass;
    mat3 inertia_tensor;
    mat3 inverse_inertia_tensor;

    mat3 inertia_tensor_world;
    mat3 inverse_inertia_tensor_world;

    /* Collision */
    int collision_group;
    // Collider * collider;

public:
    BodyType type;
    vec3 position;
    quat orientation;
    vec3 prev_position;
    quat prev_orientation;
    /* Vectors for storing the previous positions, orientations and velocities*/
    vec3 prev_linear_velocity;
    vec3 prev_angular_velocity;

    std::shared_ptr<hpp::fcl::CollisionGeometry> collider_info;

    std::optional<std::string> visual_object_path = std::nullopt;

    rs::Color color = {.r = 255, .g = 95, .b = 31, .a = 255};
    /* linear and angular velocity */
    vec3 linear_velocity;
    vec3 angular_velocity;

    /* Material properties */
    scalar dynamic_fricction_coeff = 0.5;
    scalar static_fricction_coeff = 0.5;
    /*
     *
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
        BodyType type = DYNAMIC);

    scalar get_positional_generalized_inverse_mass(vec3 r, vec3 n);

    scalar get_rotational_generalized_inverse_mass(vec3 n);

    void update_inertia_tensor_world();
    // void update_inverse_inertia_tensor_world();

    void update_position_and_orientation(scalar time_step);

    /*
     * Updates the linear and angular velocities of the body
     *
     */
    void update_velocities(scalar inverse_time_step);

    /*
     * Sets the gravity acceleration of the body
     * @param gravity The gravity acceleration that acts over the body
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
     * Applies a positional constraint impulse to the body
     * @param impulse The impulse to apply
     * @param r The position of the constraint relative to the body center (In world coordinates i.e rotated by the orientation of the body)
     */
    void apply_positional_constraint_impulse(vec3 impulse, vec3 r);

    /*
     * Applies a rotational constraint impulse to the body
     * @param impulse The impulse to apply
     */
    void apply_rotational_constraint_impulse(vec3 impulse);

    void apply_positional_velocity_constraint_impulse(vec3 impulse, vec3 r);

    void set_box_collider(vec3 half_extents, bool recompute_inertia = true);

    void set_sphere_collider(scalar radius, bool recompute_inertia = true);

    void set_capsule_collider(scalar radius, scalar height, bool recompute_inertia = true);

    void set_cylinder_collider(scalar radius, scalar height, bool recompute_inertia = true);

    void set_plane_collider(vec3 normal, scalar offset);

    void set_intertia_tensor(const mat3 &intertia_tensor);

    void set_visual_object_path(std::string path);
};