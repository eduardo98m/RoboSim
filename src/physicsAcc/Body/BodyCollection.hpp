#pragma once
#include "physics/math/math.hpp"
#include <vector>

/**
 * @brief Enum for the types of bodies that can be
 */
enum BodyType
{
    STATIC,
    DYNAMIC
};

/**
 * @brief Strcut to contain the information of a group (Collection) of bodies
 */
struct BodyCollection
{
    size_t n_bodies = 0;
    std::vector<vec3> force = {};
    std::vector<vec3> torque = {};
    std::vector<scalar> mass = {};
    std::vector<scalar> inverse_mass = {};
    std::vector<mat3> inertia_tensor = {};
    std::vector<mat3> inverse_inertia_tensor = {};
    std::vector<mat3> inertia_tensor_world = {};
    std::vector<mat3> inverse_inertia_tensor_world = {};
    std::vector<BodyType> type = {};
    std::vector<vec3> position = {};
    std::vector<quat> orientation = {};
    std::vector<vec3> linear_velocity = {};
    std::vector<vec3> angular_velocity = {};
    std::vector<vec3> prev_position = {};
    std::vector<quat> prev_orientation = {};
    std::vector<vec3> prev_linear_velocity = {};
    std::vector<vec3> prev_angular_velocity = {};
};

/**
 * @brief Updates the world-space inertia tensor and its inverse for a given body.
 *
 * Converts the body’s local inertia tensor to world coordinates using its
 * current orientation and updates both the inertia tensor and its inverse.
 *
 * @param bc Reference to the BodyCollection containing all bodies.
 * @param i  Index of the body to update.
 */
void update_inertia_tensor_world(BodyCollection &bc, size_t i);

/**
 * @brief Advances positions and orientations of all non-static bodies over time.
 *
 * For each dynamic body:
 * 1. Store previous pose.
 * 2. Recompute world inertia tensor.
 * 3. Integrate linear velocity from applied forces.
 * 4. Integrate position.
 * 5. Integrate angular velocity from applied torques (with gyroscopic term).
 * 6. Update orientation quaternion via small-angle approximation.
 *
 * @param bc Reference to the BodyCollection containing all bodies.
 * @param dt Time step (Δt) for integration.
 */
void update_position_and_orientation(BodyCollection &bc, scalar dt);

/**
 * @brief Recalculates linear and angular velocities based on pose changes.
 *
 * For each dynamic body:
 * 1. Store previous velocities.
 * 2. Compute new linear velocity from position delta.
 * 3. Compute new angular velocity from quaternion delta.
 *
 * @param bc    Reference to the BodyCollection containing all bodies.
 * @param inv_dt Inverse of the time step (1/Δt).
 */
void update_velocities(BodyCollection &bc, scalar inv_dt);

/**
 * @brief Applies an instantaneous positional impulse to resolve constraints.
 *
 * Moves the body’s center of mass by the impulse scaled by inverse mass,
 * and applies the corresponding small rotation from the impulse about point r.
 *
 * @param bc      Reference to the BodyCollection.
 * @param i       Index of the body to which the impulse is applied.
 * @param impulse Impulse vector in world coordinates.
 * @param r       Lever arm (vector) from the center of mass to the application point.
 */
void apply_positional_constraint_impulse(BodyCollection &bc, size_t i, vec3 impulse, vec3 r);

/**
 * @brief Applies an instantaneous rotational impulse to resolve constraints.
 *
 * Rotates the body by the small-angle approximation corresponding to
 * the given impulse in world coordinates.
 *
 * @param bc      Reference to the BodyCollection.
 * @param i       Index of the body to which the impulse is applied.
 * @param impulse Rotational impulse vector (torque impulse) in world coordinates.
 */
void apply_rotational_constraint_impulse(BodyCollection &bc, size_t i, vec3 impulse);

/**
 * @brief Applies an impulse directly to a body's velocity.
 *
 * This function enforces a velocity‐level positional constraint by adjusting
 * both linear and angular velocities based on a specified impulse applied
 * at an offset from the center of mass.
 *
 * @param bc      The BodyCollection containing body states.
 * @param i       Index of the body to which the impulse is applied.
 * @param impulse Impulse vector in world coordinates.
 * @param r       Lever arm from the body's center of mass to the application point, in world coordinates.
 *
 * If the body is static (type == BodyType::STATIC), no changes are made.
 * Otherwise:
 * - The body's linear velocity is incremented by impulse * inverse_mass.
 * - The body's angular velocity is incremented by inverse_inertia_tensor_world × (r × impulse).
 */
void apply_positional_velocity_constraint_impulse(BodyCollection &bc, size_t i, vec3 impulse, vec3 r);
/**
 * @brief Computes the generalized inverse mass for positional constraint resolution.
 *
 * This scalar combines the body’s inverse mass and inverse inertia
 * to measure how responsive it is to an impulse at point r in direction n.
 *
 * @param bc Reference to the BodyCollection.
 * @param i  Index of the body.
 * @param r  Vector from center of mass to contact point.
 * @param n  Constraint direction (unit normal).
 * @return   Generalized inverse mass (zero if the body is static).
 */
scalar get_positional_generalized_inverse_mass(BodyCollection &bc, size_t i, vec3 r, vec3 n);

/**
 * @brief Computes the generalized inverse mass for rotational constraint resolution.
 *
 * Measures how responsive the body is to a pure rotational impulse about axis n.
 *
 * @param bc Reference to the BodyCollection.
 * @param i  Index of the body.
 * @param n  Rotation axis (unit direction).
 * @return   Generalized inverse mass (zero if the body is static).
 */
scalar get_rotational_generalized_inverse_mass(BodyCollection &bc, size_t i, vec3 n);