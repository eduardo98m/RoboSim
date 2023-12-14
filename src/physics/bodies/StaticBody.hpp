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
};