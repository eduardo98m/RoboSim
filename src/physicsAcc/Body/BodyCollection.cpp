#include "physicsAcc/Body/BodyCollection.hpp"


void update_inertia_tensor_world(BodyCollection &bc, size_t i)
{
    mat3 rotation_matrix = ti::mat3_cast(bc.orientation[i]);
    bc.inertia_tensor_world[i] = rotation_matrix * bc.inertia_tensor[i] * ti::transpose(rotation_matrix);
    bc.inverse_inertia_tensor_world[i] = rotation_matrix * bc.inverse_inertia_tensor[i] * ti::transpose(rotation_matrix);
}


void update_position_and_orientation(BodyCollection &bc, scalar dt)
{

    for (size_t i = 0; i < bc.n_bodies; i++)
    {
        if (bc.type[i] == BodyType::STATIC)
            return;
        bc.prev_position[i] = bc.position[i];
        bc.prev_orientation[i] = bc.orientation[i];
        update_inertia_tensor_world(bc, i);
        bc.linear_velocity[i] += bc.force[i] * bc.inverse_mass[i] * dt;
        bc.position[i] += bc.linear_velocity[i] * dt;
        bc.angular_velocity[i] += dt * bc.inverse_inertia_tensor_world[i] *
                                  (bc.torque[i] - ti::cross(bc.angular_velocity[i], bc.inertia_tensor_world[i] * bc.angular_velocity[i]));
        vec3 omega_scaled = 0.5 * dt * bc.angular_velocity[i];
        quat rot = quat(0.0, omega_scaled.x, omega_scaled.y, omega_scaled.z);
        bc.orientation[i] += rot * bc.orientation[i];
        bc.orientation[i] = ti::normalize(bc.orientation[i]);
    }
}

void update_velocities(BodyCollection &bc, scalar inv_dt)
{

    for (size_t i = 0; i < bc.n_bodies; i++)
    {
        if (bc.type[i] == BodyType::STATIC) return;
        bc.prev_linear_velocity[i] = bc.linear_velocity[i];
        bc.prev_angular_velocity[i] = bc.angular_velocity[i];
        bc.linear_velocity[i] = (bc.position[i] - bc.prev_position[i]) * inv_dt;
        quat dq = bc.orientation[i] * ti::conjugate(bc.prev_orientation[i]);
        bc.angular_velocity[i] = 2 * inv_dt * vec3{dq.x, dq.y, dq.z};
        bc.angular_velocity[i] = dq.w >= 0 ? bc.angular_velocity[i] : -bc.angular_velocity[i];
    }
}

void apply_positional_constraint_impulse(BodyCollection &bc, size_t i, vec3 impulse, vec3 r){
    if (bc.type[i] == BodyType::STATIC) return;
    bc.position[i] += impulse * bc.inverse_mass[i];
    vec3 rot = bc.inverse_inertia_tensor_world[i] * ti::cross(r, impulse) * 0.5;
    quat rot_quat = quat(0.0, rot.x, rot.y, rot.z);
    bc.orientation[i] += rot_quat * bc.orientation[i];
    bc.orientation[i] = ti::normalize(bc.orientation[i]);
}

void apply_rotational_constraint_impulse(BodyCollection &bc, size_t i, vec3 impulse){
    if (bc.type[i] == BodyType::STATIC) return;
    vec3 rot = bc.inverse_inertia_tensor_world[i] * impulse * 0.5;
    quat rot_quat = quat(0.0, rot.x, rot.y, rot.z);
    bc.orientation[i] += rot_quat * bc.orientation[i];
    bc.orientation[i] = ti::normalize(bc.orientation[i]);
}

scalar get_positional_generalized_inverse_mass(BodyCollection &bc, size_t i, vec3 r, vec3 n)
{
    if (bc.type[i] == BodyType::STATIC) return 0.0;
    update_inertia_tensor_world(bc, i);
    vec3 cross_r_n = ti::cross(r, n);
    return bc.inverse_mass[i] + ti::dot(cross_r_n, bc.inverse_inertia_tensor_world[i] * cross_r_n);
}

scalar get_rotational_generalized_inverse_mass(BodyCollection &bc, size_t i, vec3 n){
    if (bc.type[i] == BodyType::STATIC) return 0.0;
    update_inertia_tensor_world(bc, i);
    return ti::dot(n, bc.inverse_inertia_tensor_world[i] * n);
}