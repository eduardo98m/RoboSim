#pragma once

#include "physics/math/math.hpp"
#include "Body.hpp"

// The DynamicBody class inherits from the Body class
class DynamicBody : public Body
{
    private:
        /* Vectors for storing the previous positions, orientations and velocities*/
        vec3 prev_position;
        quat prev_orientation;
        vec3 prev_linear_velocity;
        vec3 prev_angular_velocity;
        /* linear and angular velocity */
        vec3 linear_velocity;
        vec3 angular_velocity;

        /* linear and angular force */
        vec3 force;
        vec3 torque;

        /* Mass properties */
        scalar mass;
        scalar inverse_mass;
        mat3 inertia_tensor;
        mat3 inverse_inertia_tensor;

        mat3 inertia_tensor_world;
        mat3 inverse_inertia_tensor_world;

        /* Collision */
        int collision_group;
        //Collider * collider;

    public:
        DynamicBody(
                vec3 position = vec3(0.0, 0.0, 0.0),
                quat orientation = quat(1.0, 0.0, 0.0, 0.0),
                vec3 linear_velocity = vec3(0.0, 0.0, 0.0),
                vec3 angular_velocity = vec3(0.0, 0.0, 0.0),
                scalar mass = 1.0,
                mat3 inertia_tensor = mat3(1.0, 0.0, 0.0,
                                            0.0, 1.0, 0.0,
                                            0.0, 0.0, 1.0));
        
        ~DynamicBody();
        scalar get_positional_generalized_inverse_mass(vec3 r, vec3 n);

        scalar get_rotational_generalized_inverse_mass(vec3 n);
        
        void update_inertia_tensor_world();
        void update_inverse_inertia_tensor_world();
        
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
        
};