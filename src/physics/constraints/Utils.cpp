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
