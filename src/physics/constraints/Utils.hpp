#pragma once
#include "physics/math/math.hpp"


struct angle_limit_constraint_response
{
    scalar angle;
    vec3 delta_q;
};


/**
* @brief Function tha calculates the angle limit constraint value, note that the angle phi is given as a reference, this is so the functions corrects its value
* @param n Common rotation axis
* @param n_1 Rotation axis of object 1
* @param n_2 Rotation axis of object 2
* @param upper_limit Upper limit
*/
angle_limit_constraint_response compute_angle_limit_constraint_value(scalar phi,
                               vec3 n,
                               vec3 n_1,
                               vec3 n_2,
                               scalar lower_limit,
                               scalar upper_limit);

//scalar calculate_angle(vec3 n, vec3 n_1, vec3 n_2);