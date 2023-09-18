#pragma once

#include "physics/math/math.hpp"
#include "Body.hpp"

// The StaticBody class inherits from the Body class
class StaticBody : public Body
{
    private:

        /* Collision */
        int collision_group;
        //Collider * collider;

    public:
        StaticBody( vec3 position,
                    quat orientation);
        
        ~StaticBody();
        /*
        * Applies a positional constraint impulse to the body
        * as the body is static, this method does nothing 
        * but it is needed for the constraint solver
        * @param impulse The impulse to apply
        * @param r The position of the constraint relative to the body center 
        * (In world coordinates i.e rotated by the orientation of the body)
        */
        void apply_positional_constraint_impulse(vec3 impulse, vec3 r);
};