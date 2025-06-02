//BodyAPI.hpp
#pragma once

#include "physicsAcc/Body/BodyCollection.hpp"

struct BodyParams {
    BodyType type = DYNAMIC;
    scalar mass = 1.0;
    mat3 inertia_tensor = mat3{1.0, 0.0, 0.0, 
                                0.0, 1.0, 0.0, 
                                0.0, 0.0, 1.0}; // Default to identity
    vec3 position = vec3(0.0);
    quat orientation = ti::quat_from_axis_angle({1.0, 0.0, 0.0}, 0.0); // Default to identity
    vec3 linear_velocity = vec3(0.0);
    vec3 angular_velocity = vec3(0.0);
};


/**
 * @brief Creates a new body and adds it to the BodyCollection.
 *
 * This function creates a new body based on the provided parameters and adds it
 * to the BodyCollection. It initializes all the necessary properties of the body.
 *
 * @param bc The BodyCollection to add the new body to.
 * @param params The parameters defining the properties of the new body.
 * @return The ID of the created body (its index in the BodyCollection).
 */
size_t create_body(BodyCollection &bc, BodyParams params) {
    size_t id = bc.n_bodies;
    bc.n_bodies += 1;

    bc.type.push_back(params.type);
    bc.mass.push_back(params.mass);
    bc.inverse_mass.push_back(params.type == STATIC ? 0.0 : 1.0 / params.mass); // Static bodies have infinite mass
    bc.inertia_tensor.push_back(params.inertia_tensor);
    bc.inverse_inertia_tensor.push_back(ti::inverse(params.inertia_tensor));
    bc.inertia_tensor_world.push_back(params.inertia_tensor); // Initialize with local values
    bc.inverse_inertia_tensor_world.push_back(ti::inverse(params.inertia_tensor)); // Initialize with local values
    bc.position.push_back(params.position);
    bc.orientation.push_back(params.orientation);
    bc.linear_velocity.push_back(params.linear_velocity);
    bc.angular_velocity.push_back(params.angular_velocity);
    bc.force.push_back({0.0, -9.8*params.mass, 0.0});        // Initialize force to zero
    bc.torque.push_back(vec3(0.0));       // Initialize torque to zero
    bc.prev_position.push_back(params.position);
    bc.prev_orientation.push_back(params.orientation);
    bc.prev_linear_velocity.push_back(params.linear_velocity);
    bc.prev_angular_velocity.push_back(params.angular_velocity);

    return id;
}
