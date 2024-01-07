#include "Utils.hpp"


angle_limit_constraint_response compute_angle_limit_constraint_value(scalar phi,
                               vec3 n,
                               vec3 n_1,
                               vec3 n_2,
                               scalar lower_limit,
                               scalar upper_limit)
{

    vec3 delta_q = {0.0, 0.0, 0.0};

    if (phi < lower_limit || phi > upper_limit)
    {

        phi = ti::clamp(phi, lower_limit, upper_limit);

        quat rot =  ti::quat_from_axis_angle(n, phi);

        n_1 = ti::rotate(rot, n_1); 

        delta_q = ti::cross(n_1, n_2);
    }

    return {phi , delta_q};
}

// NOT REALLY USED (WE PREFFER TO USE ARCTAN2)
// scalar calculate_angle(vec3 n, vec3 n_1, vec3 n_2){

//     scalar phi = asin(ti::dot(ti::cross(n_1, n_2), n));
    
//     if (ti::dot(n_1, n_2)) phi = PI - phi;
//     if (phi > PI) phi = phi - 2 * PI;
//     if (phi < - PI) phi = phi + 2 * PI;

//     return phi;
// }
