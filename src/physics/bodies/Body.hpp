#pragma once
#include "physics/math/math.hpp"

class Body
{
private:
    /* position and orientation */
    
public:
    vec3 position;
    quat orientation;

    Body(
        vec3 position,
        quat orientation
        );
    ~Body();

    /*Methods for get the generalized inverse mass*/
    scalar get_positional_generalized_inverse_mass(vec3 r, vec3 n);
    scalar get_rotational_generalized_inverse_mass();

    /*
    * Applies a positional constraint impulse to the body
    * @param impulse The impulse to apply
    * @param r The position of the constraint relative to the body center (In world coordinates i.e rotated by the orientation of the body)
    */
    void apply_positional_constraint_impulse(vec3 impulse, vec3 r);
    
};
