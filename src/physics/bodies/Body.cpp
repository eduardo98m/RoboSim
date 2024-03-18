#include "Body.hpp"
#include "physics/math/math.hpp"

Body::Body(
    vec3 position,
    quat orientation,
    vec3 linear_velocity ,
    vec3 angular_velocity,
    scalar mass,
    mat3 inertia_tensor,
    BodyType body_type) 
{   
    // Set the position and orientation
    this->position = position;
    this->orientation = orientation;

    this->orientation = orientation;
    // Set the type
    this->type = body_type;
    // Set the mass
    this->mass = mass;
    // Set the inverse mass
    this->inverse_mass = 1.0 / mass;
    // Set the inertia tensor
    this->inertia_tensor = inertia_tensor;
    // Set the inverse inertia tensor
    this->inverse_inertia_tensor = ti::inverse(this->inertia_tensor);
    // Set the inertia tensor world
    this->inertia_tensor_world = this->inertia_tensor;
    // Set the inverse inertia tensor world
    this->inverse_inertia_tensor_world = this->inverse_inertia_tensor;
    // Set the linear velocity
    this->linear_velocity = linear_velocity;
    // Set the angular velocity
    this->angular_velocity = angular_velocity;
    // Set the previous position
    this->prev_position = position;
    // Set the previous orientation
    this->prev_orientation = orientation;
    // Set the previous linear velocity
    this->prev_linear_velocity = linear_velocity;
    // Set the previous angular velocity
    this->prev_angular_velocity = angular_velocity;
}


void Body::set_gravity(vec3 gravity) 
{   
    this->force += gravity * this->mass;
}

scalar Body::get_positional_generalized_inverse_mass(vec3 r, vec3 n)
{
    if (this->type == BodyType::STATIC) return 0.0;
    this->update_inertia_tensor_world();
    vec3 cross_r_n = ti::cross(r, n);
    return this->inverse_mass + ti::dot(cross_r_n, this->inverse_inertia_tensor_world * cross_r_n);
}

scalar Body::get_rotational_generalized_inverse_mass(vec3 n){
    this->update_inertia_tensor_world();
    return ti::dot(n, this->inverse_inertia_tensor_world * n);
}

void Body::update_inertia_tensor_world()
{
    mat3 rotation_matrix = ti::mat3_cast(this->orientation);
    this->inertia_tensor_world = rotation_matrix * this->inertia_tensor * ti::transpose(rotation_matrix);
    this->inverse_inertia_tensor_world = rotation_matrix * this->inverse_inertia_tensor * ti::transpose(rotation_matrix);
}

void Body::update_position_and_orientation(scalar time_step)
{
    if (this->type == BodyType::STATIC) return;
    
    // Save the previous position and orientation
    this->prev_position = this->position;
    this->prev_orientation = this->orientation;

    this->update_inertia_tensor_world();

    // Update the velocity
    this->linear_velocity += this->force * this->inverse_mass * time_step;

    // Update the position
    this->position += this->linear_velocity * time_step;

    // Update the angular velocity
    this->angular_velocity += time_step * this->inverse_inertia_tensor_world *
                              (this->torque - ti::cross(this->angular_velocity, this->inertia_tensor_world * this->angular_velocity));

    // Update the orientation using the linearized formula
    // Calculate the scaled angular velocity
    vec3 omega_scaled = 0.5 * time_step * this->angular_velocity;
    // Calculate the scaled quaternion
    quat rot = quat(0.0, omega_scaled.x, omega_scaled.y, omega_scaled.z);
    // Update the orientation
    this->orientation += rot * this->orientation;
    // Normalize the orientation
    this->orientation = ti::normalize(this->orientation);   
}

void Body::update_velocities(scalar inverse_time_step)
{

    if (this->type == BodyType::STATIC) return;

    // Save the previous linear and angular velocities
    this->prev_linear_velocity = this->linear_velocity;
    this->prev_angular_velocity = this->angular_velocity;
    // Update the linear velocity
    this->linear_velocity = (this->position - this->prev_position) * inverse_time_step;
    // Update the angular velocity || dq : delta orientation
    quat dq = this->orientation * ti::conjugate(this->prev_orientation);
    // Calculate the scaled angular velocity
    this->angular_velocity = 2 * inverse_time_step * vec3{dq.x, dq.y, dq.z};
    // Set the angular velocity sign according to the delta orientation (dq.w >= 0 ? 1 : -1)
    this->angular_velocity = dq.w >= 0 ? this->angular_velocity : -this->angular_velocity;
}

void Body::apply_positional_constraint_impulse(vec3 impulse, vec3 r){

    if (this->type == BodyType::STATIC) return;

    this->position += impulse * this->inverse_mass;
    vec3 rot = this->inverse_inertia_tensor_world * ti::cross(r, impulse) * 0.5;
    // Calculate the scaled quaternion
    quat rot_quat = quat(0.0, rot.x, rot.y, rot.z);
    // Update the orientation
    this->orientation += rot_quat * this->orientation;
    // Normalize the orientation
    this->orientation = ti::normalize(this->orientation);
}

void Body::apply_rotational_constraint_impulse(vec3 impulse){

    if (this->type == BodyType::STATIC) return;

    vec3 rot = this->inverse_inertia_tensor_world * impulse * 0.5;

    // Calculate the scaled quaternion
    quat rot_quat = quat(0.0, rot.x, rot.y, rot.z);

    // Update the orientation
    this->orientation += rot_quat * this->orientation;
    // Normalize the orientation
    this->orientation = ti::normalize(this->orientation);

}

void Body::apply_positional_velocity_constraint_impulse(vec3 impulse, vec3 r){

    if (this->type == BodyType::STATIC) return;
    // Update the orientation
    this->linear_velocity += impulse * this->inverse_mass;

    this->angular_velocity += this->inverse_inertia_tensor_world  * ti::cross(r, impulse);

}

void Body::set_intertia_tensor(const mat3 &intertia_tensor){
    
    this->inertia_tensor = inertia_tensor;
    this->inverse_inertia_tensor = ti::inverse(this->inertia_tensor);
    this->update_inertia_tensor_world();
}

void Body::set_box_collider(vec3 half_extents, bool recompute_inertia){
    this->collider_info = std::make_shared<hpp::fcl::Box>(
         half_extents.x * 2.0, 
         half_extents.y * 2.0, 
        half_extents.z * 2.0);

    if (recompute_inertia && this->type == BodyType::DYNAMIC){
        scalar vol = this->collider_info->computeVolume();
        inertia_tensor = ti::mar3_from_eigen(this->collider_info->computeMomentofInertiaRelatedToCOM())/vol;
        set_intertia_tensor(inertia_tensor * this->mass);
    }
    
}

void Body::set_sphere_collider(scalar radius, bool recompute_inertia){
    this->collider_info =  std::make_shared<hpp::fcl::Sphere>(radius);
    if (recompute_inertia && this->type == BodyType::DYNAMIC){
        scalar vol = this->collider_info->computeVolume();
        inertia_tensor = ti::mar3_from_eigen(this->collider_info->computeMomentofInertiaRelatedToCOM())/vol;
        set_intertia_tensor(inertia_tensor * this->mass);
    }
}

void Body::set_capsule_collider(scalar radius, scalar height, bool recompute_inertia){
    this->collider_info =  std::make_shared<hpp::fcl::Capsule>(  radius,  height);
    if (recompute_inertia && this->type == BodyType::DYNAMIC){
        scalar vol = this->collider_info->computeVolume();
        inertia_tensor = ti::mar3_from_eigen(this->collider_info->computeMomentofInertiaRelatedToCOM())/vol;
        set_intertia_tensor(inertia_tensor * this->mass);
    }
}

void Body::set_cylinder_collider(scalar radius, scalar height, bool recompute_inertia){
    this->collider_info =  std::make_shared<hpp::fcl::Cylinder>( radius,  height);
    if (recompute_inertia && this->type == BodyType::DYNAMIC){
        scalar vol = this->collider_info->computeVolume();
        inertia_tensor = ti::mar3_from_eigen(this->collider_info->computeMomentofInertiaRelatedToCOM())/vol;
        set_intertia_tensor(inertia_tensor * this->mass);
    }
}

void Body::set_plane_collider(vec3 normal, scalar offset){
    this->collider_info =  std::make_shared<hpp::fcl::Halfspace>(normal.x, normal.y, normal.z, offset);
}
